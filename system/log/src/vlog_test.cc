/*
 * Copyright 2023 The Android Open Source Project
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

#define LOG_TAG "test"

#include <gtest/gtest.h>
#include <log/log.h>

#include "bluetooth/log.h"
#include "truncating_buffer.h"

/// Captures the latest message generated by the android vlog
/// implementation.
static std::optional<__android_log_message> androidLogMessage;

/// Mask the implementation from liblog.
int __android_log_is_loggable(int /*prio*/, const char* /*tag*/,
                              int /*default_prio*/) {
  return true;
}

/// Mask the implementation from liblog.
void __android_log_write_log_message(
    struct __android_log_message* log_message) {
  if (log_message != nullptr) {
    log_message->message = strdup(log_message->message);
    androidLogMessage.emplace(*log_message);
  }
}

using namespace bluetooth;

TEST(BluetoothLoggerTest, verbose) {
  androidLogMessage.reset();

  log::verbose("verbose test");

  ASSERT_TRUE(androidLogMessage.has_value());
  EXPECT_EQ(androidLogMessage->priority, ANDROID_LOG_VERBOSE);
  EXPECT_STREQ(androidLogMessage->tag, LOG_TAG);
  EXPECT_STREQ(androidLogMessage->file,
               "packages/modules/Bluetooth/system/log/src/vlog_test.cc");
  EXPECT_EQ(androidLogMessage->line, 49);
  EXPECT_STREQ(androidLogMessage->message, "TestBody: verbose test");
}

TEST(BluetoothLoggerTest, debug) {
  androidLogMessage.reset();

  log::debug("debug test");

  ASSERT_TRUE(androidLogMessage.has_value());
  EXPECT_EQ(androidLogMessage->priority, ANDROID_LOG_DEBUG);
  EXPECT_STREQ(androidLogMessage->tag, LOG_TAG);
  EXPECT_STREQ(androidLogMessage->file,
               "packages/modules/Bluetooth/system/log/src/vlog_test.cc");
  EXPECT_EQ(androidLogMessage->line, 63);
  EXPECT_STREQ(androidLogMessage->message, "TestBody: debug test");
}

TEST(BluetoothLoggerTest, info) {
  androidLogMessage.reset();

  log::info("info test");

  ASSERT_TRUE(androidLogMessage.has_value());
  EXPECT_EQ(androidLogMessage->priority, ANDROID_LOG_INFO);
  EXPECT_STREQ(androidLogMessage->tag, LOG_TAG);
  EXPECT_STREQ(androidLogMessage->file,
               "packages/modules/Bluetooth/system/log/src/vlog_test.cc");
  EXPECT_EQ(androidLogMessage->line, 77);
  EXPECT_STREQ(androidLogMessage->message, "TestBody: info test");
}

TEST(BluetoothLoggerTest, warn) {
  androidLogMessage.reset();

  log::warn("warn test");

  ASSERT_TRUE(androidLogMessage.has_value());
  EXPECT_EQ(androidLogMessage->priority, ANDROID_LOG_WARN);
  EXPECT_STREQ(androidLogMessage->tag, LOG_TAG);
  EXPECT_STREQ(androidLogMessage->file,
               "packages/modules/Bluetooth/system/log/src/vlog_test.cc");
  EXPECT_EQ(androidLogMessage->line, 91);
  EXPECT_STREQ(androidLogMessage->message, "TestBody: warn test");
}

TEST(BluetoothLoggerTest, error) {
  androidLogMessage.reset();

  log::error("error test");

  ASSERT_TRUE(androidLogMessage.has_value());
  EXPECT_EQ(androidLogMessage->priority, ANDROID_LOG_ERROR);
  EXPECT_STREQ(androidLogMessage->tag, LOG_TAG);
  EXPECT_STREQ(androidLogMessage->file,
               "packages/modules/Bluetooth/system/log/src/vlog_test.cc");
  EXPECT_EQ(androidLogMessage->line, 105);
  EXPECT_STREQ(androidLogMessage->message, "TestBody: error test");
}
