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

class OTAManager {
  bool m_is_running{false};
  esp_ota_handle_t m_update_handle{0};
  esp_partition_t const* m_update_partition{nullptr};
  std::size_t m_firmware_size{0};

 public:
  constexpr OTAManager() noexcept = default;

  OTAManager(OTAManager const&) noexcept = delete;
  auto operator=(OTAManager const&) noexcept -> OTAManager& = delete;

  OTAManager(OTAManager&&) noexcept = delete;
  auto operator=(OTAManager&&) noexcept -> OTAManager& = delete;

  constexpr ~OTAManager() noexcept = default;

  [[nodiscard]] auto start_update(std::size_t const firmware_size) -> bool {
    if (m_is_running) [[unlikely]] {
      return false;
    }

    m_update_partition = esp_ota_get_next_update_partition(nullptr);
    if (m_update_partition == nullptr) [[unlikely]] {
      return false;
    }

    m_firmware_size = firmware_size;

    if (esp_ota_begin(m_update_partition, m_firmware_size, &m_update_handle) != ESP_OK) [[unlikely]] {
      return false;
    }

    m_is_running = true;

    return true;
  }

  template <std::size_t PayloadSize>
    requires(PayloadSize > 0)
  [[nodiscard]] auto write_chunk(std::size_t const chunk_index, std::array<std::uint8_t, PayloadSize> const& data) -> bool {
    static constexpr std::size_t payload_size{PayloadSize};

    if (!m_is_running) [[unlikely]] {
      return false;
    }

    std::size_t const offset_size = chunk_index * payload_size;

    if (offset_size >= m_firmware_size) [[unlikely]] {
      return false;
    }

    if (esp_ota_write_with_offset(m_update_handle, data.data(), payload_size, offset_size) != ESP_OK) [[unlikely]] {
      return false;
    }

    return true;
  }

  [[nodiscard]] auto end_update() -> bool {
    if (!m_is_running) [[unlikely]] {
      return false;
    }

    if (esp_ota_end(m_update_handle) != ESP_OK) [[unlikely]] {
      esp_ota_abort(m_update_handle);
      m_is_running = false;
      return false;
    }

    if (esp_ota_set_boot_partition(m_update_partition) != ESP_OK) [[unlikely]] {
      esp_ota_abort(m_update_handle);
      m_is_running = false;
      return false;
    }

    m_is_running = false;

    return true;
  }

  [[noreturn]] static auto reboot() -> void { esp_restart(); }

  [[nodiscard]] auto is_active() const -> bool { return m_is_running; }
};
