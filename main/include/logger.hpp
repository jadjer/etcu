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
};
