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
