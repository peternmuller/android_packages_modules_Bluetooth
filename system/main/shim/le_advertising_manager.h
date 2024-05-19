/*
 * Copyright 2020 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/**
 * Gd shim layer to legacy le advertiser
 */
#pragma once

#include "include/hardware/ble_advertiser.h"
#include "stack/include/gap_api.h"

namespace bluetooth {
namespace shim {
class EncKeyMaterialInterface {
 public:
  virtual ~EncKeyMaterialInterface() = default;
  /** Registers an EncKeyMaterialInterface with the stack */
  virtual void Init() = 0;
  virtual void GetEncKeyMaterial() = 0;
};
BleAdvertiserInterface* get_ble_advertiser_instance();
void GetEncKeyMaterial();
namespace legacy {
void OnGetEncKeyMaterial(std::vector<uint8_t> temp, uint8_t attr_uuid);
}
void init_advertising_manager();
EncKeyMaterialInterface* get_enc_key_material_instance();
void init_enc_key_material_manager();

}  // namespace shim
}  // namespace bluetooth
