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
 * Gd shim layer to legacy le scanner
 */
#pragma once

#include "hci/address.h"
#include "include/hardware/ble_scanner.h"

namespace bluetooth {
namespace shim {
namespace legacy {
hci::Address identity_to_pseudo_random(hci::Address address,
                                       uint8_t address_type, bool refresh);
}

::BleScannerInterface* get_ble_scanner_instance();
void init_scanning_manager();
bool is_ad_type_filter_supported();
void set_ad_type_rsi_filter(bool enable);
void set_empty_filter(bool enable);
void set_target_announcements_filter(bool enable);

}  // namespace shim
}  // namespace bluetooth
