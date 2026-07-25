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

namespace devices {

template <uart_port_t Port, gpio_num_t Tx, gpio_num_t Rx>
class ECU {
  std::uint16_t m_rpm{0};
  std::uint16_t m_tps{0};
  std::uint16_t m_speed{0};

 public:
  auto init() noexcept -> SystemError {
    uart_config_t constexpr config = {
        .baud_rate = 10'400,
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
      return SystemError::ECUInitFault;
    }
    if (auto const err = uart_set_pin(Port, Tx, Rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); err != ESP_OK) {
      return SystemError::ECUInitFault;
    }
    if (auto const err = uart_driver_install(Port, 256, 256, 0, nullptr, 0); err != ESP_OK) {
      return SystemError::ECUInitFault;
    }

    return SystemError::None;
  }

  auto update() noexcept -> SystemError {
    m_rpm = 0;
    m_tps = 0;
    m_speed = 0;

    return SystemError::None;
  }

  [[nodiscard]] auto get_rpm() const noexcept -> RPM { return m_rpm; }

  [[nodiscard]] auto get_tps() const noexcept -> MilliVolt { return m_tps; }

  [[nodiscard]] auto get_speed() const noexcept -> Speed { return m_speed; }
};

}  // namespace devices
