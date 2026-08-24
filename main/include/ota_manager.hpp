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
// Created by jadjer on 27.07.26.
//

#pragma once

#include <esp_ota_ops.h>
#include "config/constants.hpp"

namespace update {

class OTAManager {
  bool m_is_running{false};
  esp_ota_handle_t m_update_handle{0};
  esp_partition_t const* m_update_partition{nullptr};

 public:
  [[nodiscard]] auto startUpdate(std::size_t const total_size) -> bool {
    if (m_is_running)
      return false;

    m_update_partition = esp_ota_get_next_update_partition(nullptr);
    if (m_update_partition == nullptr)
      return false;

    if (esp_err_t const error = esp_ota_begin(m_update_partition, total_size, &m_update_handle); error != ESP_OK)
      return false;

    m_is_running = true;
    return true;
  }

  [[nodiscard]] auto writeChunk(std::array<std::uint8_t, constants::bluetooth::MaxBlePayloadSize> const& data) const -> bool {
    if (!m_is_running)
      return false;

    return esp_ota_write(m_update_handle, data.data(), data.size()) == ESP_OK;
  }

  auto endUpdate() -> void {
    if (!m_is_running)
      return;

    if (esp_err_t const error = esp_ota_end(m_update_handle); error != ESP_OK)
      return;

    if (esp_err_t const error = esp_ota_set_boot_partition(m_update_partition); error != ESP_OK)
      return;

    m_is_running = false;

    esp_restart();
  }

  [[nodiscard]] auto isActive() const -> bool { return m_is_running; }
};

}  // namespace update
