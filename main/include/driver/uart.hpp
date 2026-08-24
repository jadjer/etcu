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

enum class UartPort : std::uint8_t {
  Uart0 = 0,
  Uart1 = 1,
  Uart2 = 2,
};

template <UartPort Port, int TxPin, int RxPin, int BaudRate, std::size_t BufferSize = 4096>
struct UART {
 private:
  static constexpr auto esp_port = static_cast<uart_port_t>(Port);
  static constexpr auto esp_tx = static_cast<gpio_num_t>(TxPin);
  static constexpr auto esp_rx = static_cast<gpio_num_t>(RxPin);
  static constexpr auto esp_baud_rate = static_cast<uint32_t>(BaudRate);

 public:
  constexpr explicit UART() noexcept = default;

  UART(UART const&) = delete;
  auto operator=(UART const&) -> UART& = delete;

  [[nodiscard]] auto init() const noexcept -> bool {  // NOLINT
    uart_config_t const config = {.baud_rate = BaudRate,
                                  .data_bits = UART_DATA_8_BITS,
                                  .parity = UART_PARITY_DISABLE,
                                  .stop_bits = UART_STOP_BITS_1,
                                  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                                  .rx_flow_ctrl_thresh = 104,
                                  .source_clk = UART_SCLK_DEFAULT,
                                  .flags = {}};

    if (esp_err_t const error = uart_param_config(esp_port, &config); error != ESP_OK) [[unlikely]]
      return false;

    if (esp_err_t const error = uart_set_pin(esp_port, esp_tx, esp_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); error != ESP_OK) [[unlikely]]
      return false;

    if (esp_err_t const error = uart_driver_install(esp_port, BufferSize, BufferSize, 0, nullptr, 0); error != ESP_OK) [[unlikely]]
      return false;

    return true;
  }

  [[nodiscard]] auto flush() const noexcept -> bool { return uart_flush_input(esp_port) == ESP_OK; }  // NOLINT

  template <std::size_t PackageSize>
  [[nodiscard]] auto write(std::array<std::uint8_t, PackageSize> const& data) const noexcept -> bool {  // NOLINT
    auto const write_bytes = uart_write_bytes(esp_port, data.data(), data.size());
    return write_bytes == static_cast<int>(data.size());
  }

  template <std::size_t PackageSize>
  [[nodiscard]] auto read(std::array<std::uint8_t, PackageSize>& data, std::uint16_t const timeout_ms) const noexcept -> int {  // NOLINT
    return uart_read_bytes(esp_port, data.data(), data.size(), pdMS_TO_TICKS(timeout_ms));
  }
};

}  // namespace driver
