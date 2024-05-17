/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "hci/le_scanning_decrypter.h"

#include <base/strings/string_number_conversions.h>
#include <openssl/aead.h>
#include <openssl/base.h>
#include <openssl/rand.h>

#include <memory>
#include <unordered_map>

#include "gd/storage/config_keys.h"
#include "hci/le_scanning_interface.h"
#include "hci/vendor_specific_event_manager.h"
#include "os/handler.h"
#include "os/log.h"
#include "stack/include/gap_api.h"

namespace bluetooth::hci {

/// Iterate over the advertising data, and attempts to decrypt
/// Encrypted Data AD types using the provided key material.
/// Stores the completely decrypted data in adv_data_decrypted.
/// Returns true if all encrypted data was successfully decrypted,
/// false if any Encrypted Data could not be read.
bool LeScanningDecrypter::ExtractEncryptedData(
    std::vector<uint8_t> const& adv_data,
    std::vector<uint8_t> const& enc_key_material,
    std::vector<uint8_t>* adv_data_decrypted) {
  bool is_decryption_success = true;
  bool enc_adv_data_map;

  log::debug("Encrypted Data: {}", base::HexEncode(adv_data.data(), adv_data.size()).c_str());

#ifdef ENABLE_ENC_ADV_DEBUG_LOG
  log::debug(
      "ENC_KEY_MATERIAL {}",
      base::HexEncode(enc_key_material.data(), enc_key_material.size()).c_str());
#endif

  std::vector<std::pair<std::vector<uint8_t>, std::vector<uint8_t>>> key_and_ivs;

  std::vector<uint8_t> decrypted_adv_data;

  /* Split the string from bt_config.conf file into individual 24 byte Encrypted data
     key material char values and save it in enc_key_material_vec vector */
  for (size_t i = 0; i + ENC_KEY_MATERIAL_LEN <= enc_key_material.size();) {
    std::vector<uint8_t> key(enc_key_material.begin() + i, enc_key_material.begin() + i + 16);
    std::vector<uint8_t> iv(
        enc_key_material.begin() + i + 16, enc_key_material.begin() + i + ENC_KEY_MATERIAL_LEN);
    key_and_ivs.push_back(std::pair(key, iv));
    i += ENC_KEY_MATERIAL_LEN;
  }

#ifdef ENABLE_ENC_ADV_DEBUG_LOG
  // Print the split 24 byte Encrypted data key material char values
  int enc_key_material_count = 0;
  for (auto enc_key_material : key_and_ivs) {
    enc_key_material_count++;
    log::debug(
        "Enc Data Key Vector {}: {} , {}",
        enc_key_material_count,
        base::HexEncode(enc_key_material.first.data(), enc_key_material.first.size()).c_str(),
        base::HexEncode(enc_key_material.second.data(), enc_key_material.second.size()).c_str());
  }
#endif

  // Iterate through the advertising data, and decrypt AD Encrypted Data
  // entries. The encrypted data is decrypted in place replacing the original
  // data.
  for (size_t position = 0; position < adv_data.size();) {
    size_t ad_len = adv_data[position];
    if (ad_len == 0 || (position + ad_len) > adv_data.size()) {
      break;
    }

    uint8_t ad_type = adv_data[position + 1];
    size_t prev_position = position;
    position += ad_len + 1;

    // check for ad_type != 0x31
    if (ad_type != (uint8_t)GapDataType::ENCRYPTED_ADVERTISING_DATA) {
      std::copy(
          adv_data.begin() + prev_position,
          adv_data.begin() + prev_position + ad_len + 1,
          std::back_inserter(decrypted_adv_data));
      continue;
    }
    std::vector<uint8_t> encrypted_data(
        adv_data.begin() + prev_position, adv_data.begin() + prev_position + ad_len + 1);

    // to store decrypted data temporary
    std::optional<std::vector<uint8_t>> decrypted_data = {};

    // Iterate through the multiple enc data key char values to check
    // and find the enc key which successfully decrypts the data.
    for (auto const& [key, iv] : key_and_ivs) {
#ifdef ENABLE_ENC_ADV_DEBUG_LOG
      log::debug("Session Key: {}", base::HexEncode(key.data(), key.size()).c_str());
      log::debug("IV: {}", base::HexEncode(iv.data(), iv.size()).c_str());
#endif
      decrypted_data = DecryptEncryptedData(encrypted_data, key, iv);
      if (decrypted_data.has_value()) break;
    }

    if (decrypted_data.has_value()) {
      std::copy(
          decrypted_data.value().begin(),
          decrypted_data.value().end(),
          std::back_inserter(decrypted_adv_data));

    } else {
      is_decryption_success = false;
      std::copy(
          adv_data.begin() + prev_position,
          adv_data.begin() + prev_position + ad_len + 1,
          std::back_inserter(decrypted_adv_data));
    }
  }
  *adv_data_decrypted = decrypted_adv_data;

  return is_decryption_success;
}

/// Identifies Encrypted Advertising Data in the advertising data
/// and stores the position and length of the data in a map.
bool LeScanningDecrypter::ContainsEncryptedData(const uint8_t* ad, size_t ad_len) {
  size_t position = 0;
  bool is_enc_adv = false;
  int enc_data_part_length = 0;

  while (position < ad_len) {
    uint8_t len = ad[position];

    if (len == 0) break;
    if (position + len >= ad_len) break;

    uint8_t adv_type = ad[position + 1];

    if (adv_type == (uint8_t)GapDataType::ENCRYPTED_ADVERTISING_DATA) {
      enc_data_part_length = len + 1; /* Length(1 byte) + len */
      is_enc_adv = true;
    }

    position += len + 1; /* skip the length of data */
  }

  if (!is_enc_adv) log::verbose("enc_adv_data_map is empty");

  return is_enc_adv;
}

/// This function receives the complete adv_data,
/// the map that stores the encrypted advertising data,
/// encrypted data key material, and the decrypted data map
/// that stores the decrypted data. This function initializes
/// the decryption and algorithm and performs decryption
std::optional<std::vector<uint8_t>> LeScanningDecrypter::DecryptEncryptedData(
    std::vector<uint8_t> const& adv_data,
    std::vector<uint8_t> const& key,
    std::vector<uint8_t> const& iv) {
  static const std::vector<uint8_t> ad = {0xEA};
  const EVP_AEAD_CTX* aeadCTX = EVP_AEAD_CTX_new(
      EVP_aead_aes_128_ccm_bluetooth(), key.data(), key.size(), EVP_AEAD_DEFAULT_TAG_LENGTH);
  if (aeadCTX == nullptr) return std::nullopt;
  std::vector<uint8_t> nonce;
  std::vector<uint8_t> MIC;
  std::vector<uint8_t> payload;
  std::vector<uint8_t> randomizer;

  for (size_t i = 0; i < adv_data.size(); i++) {
    if ((i >= 2) && (i <= 6)) {
      randomizer.push_back(adv_data[i]);
    } else if ((i > 6) && i < (adv_data.size() - 4)) {
      payload.push_back(adv_data[i]);
    }
    if ((i >= (adv_data.size() - 4)) && (i < adv_data.size())) {
      MIC.push_back(adv_data[i]);
    }
  }

  nonce.insert(nonce.end(), randomizer.begin(), randomizer.end());
  nonce.insert(nonce.end(), iv.rbegin(), iv.rend());

  std::vector<uint8_t> out(payload.size());
#ifdef ENABLE_ENC_ADV_DEBUG_LOG
  if (!randomizer.empty()) {
    log::debug("Randomizer: {}", base::HexEncode(randomizer.data(), randomizer.size()).c_str());
  }
  if (!nonce.empty()) {
    log::debug("Nonce: {}", base::HexEncode(nonce.data(), nonce.size()).c_str());
  }
  if (!payload.empty()) {
    log::debug("Payload: {}", base::HexEncode(payload.data(), payload.size()).c_str());
  }
  if (!MIC.empty()) {
    log::debug("MIC: {}", base::HexEncode(MIC.data(), MIC.size()).c_str());
  }
#endif
  EVP_AEAD_CTX_open_gather(
      aeadCTX,
      out.data(),
      nonce.data(),
      nonce.size(),
      payload.data(),
      payload.size(),
      MIC.data(),
      MIC.size(),
      ad.data(),
      ad.size());

  if (out.size() > 0 && (out[0] > 0)) {
    return std::make_optional(out);
  } else {
    log::info("Decryption NOT successful : {}", base::HexEncode(out.data(), out.size()).c_str());
    return std::nullopt;
  }
}

}  // namespace bluetooth::hci
