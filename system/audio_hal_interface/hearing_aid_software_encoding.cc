/*
 * Copyright 2022 The Android Open Source Project
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
 */

#include "hearing_aid_software_encoding.h"

#include "aidl/hearing_aid_software_encoding_aidl.h"
#include "hal_version_manager.h"
#include "hidl/hearing_aid_software_encoding_hidl.h"
#include "qti_hidl/hearing_aid_software_encoding.h"

namespace bluetooth {
namespace audio {
namespace hearing_aid {

// Check if new bluetooth_audio is enabled
bool is_hal_enabled() {
  if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::HIDL) {
    return hidl::hearing_aid::is_hal_2_0_enabled();
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::AIDL) {
    return aidl::hearing_aid::is_hal_enabled();
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::QTI_HIDL) {
    LOG(INFO) << __func__ << ": qti_hidl is_hal_enabled";
    return qti_hidl::hearing_aid::is_hal_2_0_enabled();
  }
  return false;
}

// Initialize BluetoothAudio HAL: openProvider
bool init(StreamCallbacks stream_cb,
          bluetooth::common::MessageLoopThread* message_loop) {
  if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::HIDL) {
    return hidl::hearing_aid::init(stream_cb, message_loop);
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::AIDL) {
    return aidl::hearing_aid::init(stream_cb, message_loop);
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::QTI_HIDL) {
    LOG(INFO) << __func__<< ": qti_hidl init";
    return qti_hidl::hearing_aid::init(stream_cb, message_loop);
  }
  return false;
}

// Clean up BluetoothAudio HAL
void cleanup() {
  if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::HIDL) {
    hidl::hearing_aid::cleanup();
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::AIDL) {
    aidl::hearing_aid::cleanup();
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::QTI_HIDL) {
    LOG(INFO) << __func__ << ": qti_hidl cleanup";
    qti_hidl::hearing_aid::cleanup();
  }
  return;
}

// Send command to the BluetoothAudio HAL: StartSession, EndSession
void start_session() {
  if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::HIDL) {
    hidl::hearing_aid::start_session();
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::AIDL) {
    aidl::hearing_aid::start_session();
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::QTI_HIDL) {
    LOG(INFO) << __func__ << ": qti_hidl start_session";
    qti_hidl::hearing_aid::start_session();
  }
  return;
}

void end_session() {
  if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::HIDL) {
    hidl::hearing_aid::end_session();
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::AIDL) {
    aidl::hearing_aid::end_session();
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::QTI_HIDL) {
    LOG(INFO) << __func__ << ": qti_hidl end_session";
    qti_hidl::hearing_aid::end_session();
  }
  return;
}

void set_remote_delay(uint16_t delay_report_ms) {
  if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::HIDL) {
    hidl::hearing_aid::set_remote_delay(delay_report_ms);
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::AIDL) {
    aidl::hearing_aid::set_remote_delay(delay_report_ms);
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::QTI_HIDL) {
    LOG(INFO) << __func__ <<": qti_hidl set_remote_delay";
    qti_hidl::hearing_aid::set_remote_delay(delay_report_ms);
  }
  return;
}

// Read from the FMQ of BluetoothAudio HAL
size_t read(uint8_t* p_buf, uint32_t len) {
  if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::HIDL) {
    return hidl::hearing_aid::read(p_buf, len);
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::AIDL) {
    return aidl::hearing_aid::read(p_buf, len);
  } else if (HalVersionManager::GetHalTransport() ==
      BluetoothAudioHalTransport::QTI_HIDL) {
    LOG(INFO) << __func__ <<": qti_hidl read";
    return qti_hidl::hearing_aid::read(p_buf, len);
  }
  return 0;
}

}  // namespace hearing_aid
}  // namespace audio
}  // namespace bluetooth
