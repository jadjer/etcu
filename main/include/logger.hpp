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

class Logger {
  static constexpr auto TAG = "THROTTLE_CORE";

 public:
  auto init() noexcept -> void { esp_log_level_set(TAG, ESP_LOG_INFO); }
  auto log_info(const char* msg) noexcept -> void { ESP_LOGI(TAG, "%s", msg); }
  auto log_warn(const char* msg) noexcept -> void { ESP_LOGW(TAG, "%s", msg); }
  auto log_error(const char* msg) noexcept -> void { ESP_LOGE(TAG, "%s", msg); }
};
