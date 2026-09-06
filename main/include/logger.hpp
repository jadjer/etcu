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
#include <format>
#include <string_view>

#include "config/constants.hpp"

class Logger {
  static constexpr auto tag{constants::system::Name.data()};
  static constexpr std::size_t max_line_size{128};

 public:
  static auto init() noexcept -> void { esp_log_level_set(tag, ESP_LOG_INFO); }

  static auto log_info(std::string_view const message) noexcept -> void { ESP_LOGI(tag, "%.*s", static_cast<int>(message.size()), message.data()); }

  static auto log_warn(std::string_view const message) noexcept -> void { ESP_LOGW(tag, "%.*s", static_cast<int>(message.size()), message.data()); }

  static auto log_error(std::string_view const message) noexcept -> void { ESP_LOGE(tag, "%.*s", static_cast<int>(message.size()), message.data()); }
  template <typename... Args>
  auto log_info(char const* const format, Args&&... args) noexcept -> void {
    std::array<char, max_line_size> buffer{};
    std::snprintf(buffer.data(), buffer.size(), format, std::forward<Args>(args)...);
    std::string_view const view{buffer.data(), buffer.size()};
    log_info(view);
  }

  template <typename... Args>
  auto log_warn(char const* const format, Args&&... args) noexcept -> void {
    std::array<char, max_line_size> buffer{};
    std::snprintf(buffer.data(), buffer.size(), format, std::forward<Args>(args)...);
    std::string_view const view{buffer.data(), buffer.size()};
    log_warn(view);
  }

  template <typename... Args>
  auto log_error(char const* const format, Args&&... args) noexcept -> void {
    std::array<char, max_line_size> buffer{};
    std::snprintf(buffer.data(), buffer.size(), format, std::forward<Args>(args)...);
    std::string_view const view{buffer.data(), buffer.size()};
    log_error(view);
  }
};
