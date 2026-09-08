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
// Created by jadjer on 6.09.26.
//

#pragma once

namespace device {

template <class Driver, class PowerEnable>
  requires concepts::UART<Driver> && concepts::GPIO<PowerEnable>
class ServoProtocol {
  Driver& m_driver_uart;
  PowerEnable& m_driver_power;

 public:
  constexpr explicit ServoProtocol(Driver& driver_uart, PowerEnable& driver_power) noexcept : m_driver_uart(driver_uart), m_driver_power(driver_power) {}

  constexpr ServoProtocol() noexcept = delete;

  ServoProtocol(ServoProtocol const&) noexcept = delete;
  auto operator=(ServoProtocol const&) noexcept -> ServoProtocol& = delete;

  ServoProtocol(ServoProtocol&&) noexcept = delete;
  auto operator=(ServoProtocol&&) noexcept -> ServoProtocol& = delete;

  constexpr ~ServoProtocol() noexcept = default;

  auto init() noexcept -> bool {
    if (!m_driver_uart.init()) [[unlikely]] {
      return false;
    }

    if (!m_driver_power.init()) [[unlikely]] {
      return false;
    }

    if (!m_driver_power.enable()) [[unlikely]] {
      return false;
    }

    return true;
  }

  template <std::size_t ParamSize>
  auto send_message(ServoMessage<ParamSize> const& message) const noexcept -> bool {
    static constexpr std::size_t packet_size{ServoMessage<ParamSize>::total_size};

    m_driver_uart.flush();

    if (int const written_bytes = m_driver_uart.write(message.to_array()); std::cmp_less(written_bytes, packet_size)) [[unlikely]] {
      m_driver_uart.flush();
      return false;
    }

    return true;
  }

  template <std::size_t PayloadSize>
  auto receive_message(ServoMessage<PayloadSize>& message) noexcept -> bool {
    static constexpr std::size_t package_size = ServoMessage<PayloadSize>::total_size;
    static constexpr std::uint16_t read_timeout_ms{10};

    std::array<std::uint8_t, package_size> response_bytes{};

    if (!m_driver_uart.read(response_bytes, read_timeout_ms)) [[unlikely]] {
      m_driver_uart.flush();
      return false;
    }

    auto const response_message = ServoMessage<PayloadSize>{response_bytes};

    if (!response_message.is_valid()) [[unlikely]] {
      m_driver_uart.flush();
      return false;
    }

    message = response_message;

    return true;
  }
};

}  // namespace device
