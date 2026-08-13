//
// Created by jadjer on 27.07.26.
//

#pragma once

#include <esp_ota_ops.h>
#include "constants.hpp"

namespace update {
using OTAHandle = esp_ota_handle_t;
using OTAPartition = esp_partition_t const*;

class OTAManager {
  bool m_is_running{false};
  OTAHandle m_update_handle{0};
  OTAPartition m_update_partition{nullptr};

 public:
  OTAManager() = default;

  auto startUpdate(std::size_t const total_size) -> bool {
    if (m_is_running)
      return false;

    m_update_partition = esp_ota_get_next_update_partition(nullptr);
    if (m_update_partition == nullptr)
      return false;

    if (esp_err_t const err = esp_ota_begin(m_update_partition, total_size, &m_update_handle); err != ESP_OK)
      return false;

    m_is_running = true;
    return true;
  }

  [[nodiscard]] auto writeChunk(std::array<std::uint8_t, constants::MAX_BLE_PAYLOAD_SIZE> const data) const -> bool {
    if (!m_is_running)
      return false;

    return (esp_ota_write(m_update_handle, data.data(), data.size()) == ESP_OK);
  }

  auto endUpdate() -> void {
    if (!m_is_running)
      return;

    if (esp_ota_end(m_update_handle) != ESP_OK)
      return;

    if (esp_ota_set_boot_partition(m_update_partition) != ESP_OK)
      return;

    m_is_running = false;

    esp_restart();
  }

  [[nodiscard]] auto isActive() const -> bool { return m_is_running; }
};

}  // namespace update
