/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#pragma once

#include <gtest/gtest_prod.h>

#include <cstdint>
#include <list>
#include <map>
#include <optional>
#include <vector>

#include "hci/address_with_type.h"
#include "hci/hci_packets.h"

/// The LE Scanning Decrypter is responsible for decrypting the any incoming
/// encrypted data over the air. We then reassemble the data after decryption takes place

namespace bluetooth::hci {

class LeScanningDecrypter {
 public:
  LeScanningDecrypter(){};
  LeScanningDecrypter(const LeScanningDecrypter&) = delete;

  bool ExtractEncryptedData(
      std::vector<uint8_t> const& adv_data,
      std::vector<uint8_t> const& enc_key_material,
      std::vector<uint8_t>* adv_data_decrypted);

  bool ContainsEncryptedData(const uint8_t* ad, size_t ad_len);

  static std::optional<std::vector<uint8_t>> DecryptEncryptedData(
      std::vector<uint8_t> const& adv_data,
      std::vector<uint8_t> const& key,
      std::vector<uint8_t> const& iv);
};

}  // namespace bluetooth::hci
