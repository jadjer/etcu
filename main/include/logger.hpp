// Copyright 2026 Pavel Suprunov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Created by jadjer on 23.07.26.
//

#pragma once

#include <esp_log.h>
#include <array>
#include <cstdio>
#include "types.hpp"

class Logger {
  static constexpr auto TAG = "THROTTLE_CORE";
  static constexpr std::size_t BufferSize = 128;

  template <typename... Args>
  void write_log(esp_log_level_t const level, const char* const format, Args&&... args) noexcept {
    std::array<char, BufferSize> buffer{};
    std::snprintf(buffer.data(), buffer.size(), format, std::forward<Args>(args)...);
    esp_log_write(level, TAG, "%s\n", buffer.data());
  }

 public:
  auto init() noexcept -> void { esp_log_level_set(TAG, ESP_LOG_INFO); }

  template <typename... Args>
  auto log_info(char const* const format, Args&&... args) noexcept -> void {
    write_log(ESP_LOG_INFO, format, std::forward<Args>(args)...);
  }

  template <typename... Args>
  auto log_warn(char const* const format, Args&&... args) noexcept -> void {
    write_log(ESP_LOG_WARN, format, std::forward<Args>(args)...);
  }

  template <typename... Args>
  auto log_error(char const* const format, Args&&... args) noexcept -> void {
    write_log(ESP_LOG_ERROR, format, std::forward<Args>(args)...);
  }

  auto log_active_errors(SystemError const errors) noexcept -> void {
    if (errors == SystemError::None) {
      log_info("System status: No Errors");
      return;
    }

    log_error("--- Active Hardware Faults ---");

    if (has_error(errors, SystemError::ServoInitFault))
      log_error(" - ServoInitFault");
    if (has_error(errors, SystemError::ServoCommsFault))
      log_error(" - ServoCommsFault");
    if (has_error(errors, SystemError::ServoOvercurrent))
      log_error(" - ServoOvercurrent");
    if (has_error(errors, SystemError::ServoOvertemp))
      log_error(" - ServoOvertemp");
    if (has_error(errors, SystemError::ServoMechanicalFault))
      log_error(" - ServoMechanicalFault");

    if (has_error(errors, SystemError::AcceleratorInitFault))
      log_error(" - AcceleratorInitFault");
    if (has_error(errors, SystemError::AcceleratorCalibrateFault))
      log_error(" - AcceleratorCalibrateFault");
    if (has_error(errors, SystemError::AcceleratorReadFault))
      log_error(" - AcceleratorReadFault");
    if (has_error(errors, SystemError::AcceleratorMismatch))
      log_error(" - AcceleratorMismatch");

    if (has_error(errors, SystemError::GuardLock))
      log_error(" - GuardLock");

    log_error("------------------------------");
  }
};
