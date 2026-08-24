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
#include <format>
#include <string_view>

class Logger {
  static constexpr std::string_view tag{"THROTTLE_CORE"};
  static constexpr std::size_t buffer_size{128};

  template <typename... Args>
  void write_log(esp_log_level_t const level, std::format_string<Args...> format, Args&&... args) noexcept {
    std::array<char, buffer_size> buffer{};

    auto const result = std::format_to_n(buffer.data(), buffer.size() - 1, format, std::forward<Args>(args)...);
    *result.out = '\0';

    esp_log_write(level, tag.data(), "%s\n", buffer.data());
  }

 public:
  auto init() noexcept -> void { esp_log_level_set(tag.data(), ESP_LOG_INFO); }  // NOLINT

  template <typename... Args>
  auto log_info(std::format_string<Args...> format, Args&&... args) noexcept -> void {
    write_log(ESP_LOG_INFO, format, std::forward<Args>(args)...);
  }

  template <typename... Args>
  auto log_warn(std::format_string<Args...> format, Args&&... args) noexcept -> void {
    write_log(ESP_LOG_WARN, format, std::forward<Args>(args)...);
  }

  template <typename... Args>
  auto log_error(std::format_string<Args...> format, Args&&... args) noexcept -> void {
    write_log(ESP_LOG_ERROR, format, std::forward<Args>(args)...);
  }

  auto log_active_errors(type::SystemError const errors) noexcept -> void {
    if (errors == type::SystemError::None) {
      log_info("System status: No Errors");
      return;
    }

    log_error("--- Active Hardware Faults ---");

    if (has_error(errors, type::SystemError::ServoInitError))
      log_error(" - ServoInitError");
    if (has_error(errors, type::SystemError::ServoCommsError))
      log_error(" - ServoCommsError");
    if (has_error(errors, type::SystemError::ServoProtocolError))
      log_error(" - ServoProtocolError");
    if (has_error(errors, type::SystemError::ServoCheckSumError))
      log_error(" - ServoCheckSumError");
    if (has_error(errors, type::SystemError::ServoReadError))
      log_error(" - ServoReadError");
    if (has_error(errors, type::SystemError::ServoWriteError))
      log_error(" - ServoWriteError");
    if (has_error(errors, type::SystemError::ServoModeError))
      log_error(" - ServoModeError");
    if (has_error(errors, type::SystemError::ServoSpeedError))
      log_error(" - ServoSpeedError");
    if (has_error(errors, type::SystemError::ServoPositionError))
      log_error(" - ServoPositionError");
    if (has_error(errors, type::SystemError::ServoCurrentError))
      log_error(" - ServoCurrentError");
    if (has_error(errors, type::SystemError::ServoTorqueError))
      log_error(" - ServoTorqueError");
    if (has_error(errors, type::SystemError::ServoOvercurrent))
      log_error(" - ServoOvercurrent");
    if (has_error(errors, type::SystemError::ServoOvertemp))
      log_error(" - ServoOvertemp");
    if (has_error(errors, type::SystemError::ServoCalibrateError))
      log_error(" - ServoCalibrateError");

    if (has_error(errors, type::SystemError::AcceleratorInitFault))
      log_error(" - AcceleratorInitFault");
    if (has_error(errors, type::SystemError::AcceleratorCalibrateFault))
      log_error(" - AcceleratorCalibrateFault");
    if (has_error(errors, type::SystemError::AcceleratorReadFault))
      log_error(" - AcceleratorReadFault");
    if (has_error(errors, type::SystemError::AcceleratorMismatch))
      log_error(" - AcceleratorMismatch");

    if (has_error(errors, type::SystemError::GuardLock))
      log_error(" - GuardLock");

    log_error("------------------------------");
  }

  auto check_and_log_errors(type::ServoError const error) noexcept -> void {  // NOLINT
    if (error == type::ServoError::None)
      return;

    if (error & type::ServoError::Voltage)
      log_error("  -> Ошибка питания! Проверьте вольтаж линии.");
    if (error & type::ServoError::AngleLimit)
      log_error("  -> Выход за программные лимиты углов.");
    if (error & type::ServoError::Overheat)
      log_error("  -> ПЕРЕГРЕВ! Дайте приводу остыть.");
    if (error & type::ServoError::Overload)
      log_error("  -> ПЕРЕГРУЗКА ТОКА (Overload)! Защита отключила мотор.");
    if (error & type::ServoError::Encoder)
      log_error("  -> Ошибка энкодера! Сбой датчика позиции.");
    if (error & type::ServoError::Driver)
      log_error("  -> Сбой драйвера! Короткое замыкание или перегрузка ключей.");
  }
};
