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
// Created by jadjer on 18.08.26.
//

#pragma once

#include <array>
#include <cstddef>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>

namespace driver {

template <std::uint8_t Port, std::uint8_t TxPin, std::uint8_t RxPin, std::size_t BaudRate, std::size_t BufferSize = 4096>
requires (Port < UART_NUM_MAX) && (TxPin < GPIO_NUM_MAX) && (RxPin < GPIO_NUM_MAX)
class UART {
  static constexpr auto esp_port{static_cast<uart_port_t>(Port)};
  static constexpr auto esp_tx{static_cast<gpio_num_t>(TxPin)};
  static constexpr auto esp_rx{static_cast<gpio_num_t>(RxPin)};
  static constexpr auto esp_baud_rate{static_cast<uint32_t>(BaudRate)};

 public:
  static auto init() noexcept -> bool {
    if (uart_is_driver_installed(esp_port))
      return true;

    if (esp_err_t const error = uart_driver_install(esp_port, BufferSize, BufferSize, 0, nullptr, 0); error != ESP_OK) [[unlikely]]
      return false;

    uart_config_t const config = {
        .baud_rate = BaudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 104,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };

    if (esp_err_t const error = uart_param_config(esp_port, &config); error != ESP_OK) [[unlikely]] {
      uart_driver_delete(esp_port);
      return false;
    }

    if (esp_err_t const error = uart_set_pin(esp_port, esp_tx, esp_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); error != ESP_OK) [[unlikely]] {
      uart_driver_delete(esp_port);
      return false;
    }

    return true;
  }

  static auto deinit() noexcept -> bool {  // NOLINT
    return uart_driver_delete(esp_port) == ESP_OK;
  }

  static auto flush() noexcept -> bool { return uart_flush_input(esp_port) == ESP_OK; }

  template <std::size_t PackageSize>
  requires (PackageSize > 0)
  static auto write(std::array<std::uint8_t, PackageSize> const& data) noexcept -> bool {
    auto const write_bytes = uart_write_bytes(esp_port, data.data(), data.size());
    return write_bytes == static_cast<int>(data.size());
  }

  template <std::size_t PackageSize>
  requires (PackageSize > 0)
  static auto read(std::array<std::uint8_t, PackageSize>& data, std::uint16_t const timeout_ms) noexcept -> int {
    return uart_read_bytes(esp_port, data.data(), data.size(), pdMS_TO_TICKS(timeout_ms));
  }
};

}  // namespace driver
