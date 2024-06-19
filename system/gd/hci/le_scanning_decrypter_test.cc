/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "hci/le_scanning_decrypter.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::_;
using ::testing::Eq;

using namespace bluetooth;
using namespace std::chrono_literals;

namespace bluetooth::hci {

class LeScanningDecrypterTest : public ::testing::Test {
 public:
  LeScanningDecrypter leScanningDecrypter_;
};

TEST_F(LeScanningDecrypterTest, decrypt_encrypted_data) {
  std::vector<uint8_t> const adv_data = {0x02, 0x01, 0x02, 0x02, 0x0A, 0xF9, 0x1A, 0x31, 0xA0,
                                         0x7D, 0x86, 0x2D, 0xA9, 0xBA, 0xBA, 0xCF, 0x6C, 0xC5,
                                         0xCA, 0xFC, 0xFF, 0xCB, 0x26, 0x07, 0x3A, 0xAF, 0x97,
                                         0xDA, 0xAA, 0x4F, 0x1A, 0x70, 0x75};

  std::vector<uint8_t> const enc_key_material = {0x86, 0x45, 0x90, 0x85, 0x84, 0x1E, 0x8E, 0x45,
                                                 0x5D, 0x3B, 0x37, 0xA7, 0x0E, 0xA8, 0x88, 0x7F,
                                                 0xD2, 0x6C, 0xC7, 0x96, 0xE3, 0xF4, 0x9F, 0x50};
  std::vector<uint8_t> adv_data_decrypted;
  bool has_encrypted_data =
      leScanningDecrypter_.ContainsEncryptedData(adv_data.data(), adv_data.size());
  ASSERT_TRUE(has_encrypted_data);
  leScanningDecrypter_.ExtractEncryptedData(adv_data, enc_key_material, &adv_data_decrypted);
  ASSERT_EQ(
      adv_data_decrypted,
      std::vector<uint8_t>({0x02, 0x01, 0x02, 0x02, 0x0A, 0xF9, 0x0F, 0x09, 0x50, 0x69, 0x6E,
                            0x65, 0x61, 0x70, 0x70, 0x6C, 0x65, 0x5F, 0x34, 0x38, 0x38, 0x30}));
}

}  // namespace bluetooth::hci
