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

#include <driver/uart.h>

namespace driver {

using Size = std::size_t;
using GPIONum = gpio_num_t;
using UARTPort = uart_port_t;
using UARTConfig = uart_config_t;

template <UARTPort port, GPIONum txPin, GPIONum rxPin, Size baudRate, Size bufferSize = 4096>
struct UART {
  [[nodiscard]] auto init() const noexcept -> bool {  // NOLINT
    UARTConfig constexpr config = {
        .baud_rate = baudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 104,
        .source_clk = UART_SCLK_DEFAULT,
        .flags =
            {
                .allow_pd = false,
                .backup_before_sleep = false,
            },
    };

    if (uart_param_config(port, &config) != ESP_OK)
      return false;

    if (uart_set_pin(port, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
      return false;

    if (uart_driver_install(port, bufferSize, bufferSize, 0, nullptr, 0) != ESP_OK)
      return false;

    return true;
  }

  [[nodiscard]] auto flush() const noexcept -> bool {  // NOLINT
    return uart_flush_input(port) == ESP_OK;
  }

  template <type::Size packageSize>
  [[nodiscard]] auto write(std::array<type::Byte, packageSize> const& data) const noexcept -> bool {  // NOLINT
    auto const write_bytes = uart_write_bytes(port, data.data(), data.size());
    return write_bytes == data.size();
  }

  template <type::Size packageSize>
  [[nodiscard]] auto read(std::array<type::Byte, packageSize>& data, type::Time const timeout) const noexcept -> int {  // NOLINT
    return uart_read_bytes(port, data.data(), data.size(), pdMS_TO_TICKS(timeout));
  }
};

}  // namespace driver
