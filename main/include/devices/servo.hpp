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

#include <driver/uart.h>
#include <array>
#include "configs/configs.hpp"
#include "types.hpp"

namespace devices {

template <uart_port_t Port, gpio_num_t Tx, gpio_num_t Rx>
class Servo {
  static Position constexpr MinimalPosition = 0;
  static Position constexpr MaximalPosition = 100;

 public:
  auto init() noexcept -> SystemError {
    uart_config_t constexpr config = {
        .baud_rate = 1'000'000,
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
    if (auto const err = uart_param_config(Port, &config); err != ESP_OK) {
      return SystemError::ServoInitFault;
    }
    if (auto const err = uart_set_pin(Port, Tx, Rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); err != ESP_OK) {
      return SystemError::ServoInitFault;
    }
    if (auto const err = uart_driver_install(Port, 256, 256, 0, nullptr, 0); err != ESP_OK) {
      return SystemError::ServoInitFault;
    }

    return SystemError::None;
  }

  auto self_test() noexcept -> SystemError { return SystemError::None; }

  auto set_position(Position const target_position) noexcept -> SystemError {
    auto const target_steps = static_cast<std::uint16_t>((static_cast<std::uint32_t>(target_position) * 4095) / 10000);

    std::array<std::uint8_t, 7> const packet = {
        0xFF, 0xFF, 0x01, 0x03, 0x2A, static_cast<uint8_t>(target_steps >> 8), static_cast<uint8_t>(target_steps & 0xFF),
    };
    int const packet_size = packet.size();

    int const written = uart_write_bytes(Port, packet.data(), packet_size);
    if (written != packet_size) {
      return SystemError::ServoCommsFault;
    }

    return SystemError::None;
  }

  auto read_telemetry(ServoTelemetry& telemetry) noexcept -> SystemError {
    int const buffer_size = 8;
    std::array<std::uint8_t, buffer_size> buffer = {};

    int const len = uart_read_bytes(Port, buffer.data(), buffer_size, 2 / portTICK_PERIOD_MS);

    if (len <= 0) {
      return SystemError::ServoCommsFault;
    }

    telemetry.position = buffer[2];
    telemetry.current = buffer[3];
    telemetry.temperature = buffer[4];

    return SystemError::None;
  }
};

}  // namespace devices
