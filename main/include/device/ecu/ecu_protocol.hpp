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

#include <utility>

#include "config/concepts.hpp"

namespace device {

template <class DriverUart, class DriverGPIO>
  requires concepts::UART<DriverUart> && concepts::GPIO<DriverGPIO>
class ECUProtocol {
  static constexpr std::uint16_t timeout_ms{100};

  DriverUart& m_driver_uart;
  DriverGPIO& m_driver_gpio;

 public:
  constexpr explicit ECUProtocol(DriverUart& driver_uart, DriverGPIO& driver_gpio) noexcept : m_driver_uart{driver_uart}, m_driver_gpio{driver_gpio} {}

  constexpr ECUProtocol() noexcept = delete;

  ECUProtocol(ECUProtocol const&) noexcept = delete;
  auto operator=(ECUProtocol const&) noexcept -> ECUProtocol& = delete;

  ECUProtocol(ECUProtocol&&) noexcept = delete;
  auto operator=(ECUProtocol&&) noexcept -> ECUProtocol& = delete;

  constexpr ~ECUProtocol() noexcept = default;

  auto init() noexcept -> bool { return m_driver_uart.init() && m_driver_gpio.init(); }

  auto wakeup() noexcept -> bool {
    static constexpr ECUMessage wakeup{0xFE, ECUMode::WAKE_UP};
    static constexpr std::uint8_t wait_low_ms{70};
    static constexpr std::uint8_t wait_high_ms{130};

    if (!m_driver_gpio.disable()) [[unlikely]] {
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(wait_low_ms));

    if (!m_driver_gpio.enable()) [[unlikely]] {
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(wait_high_ms));

    if (!m_driver_uart.init()) [[unlikely]] {
      return false;
    }

    if (!send_message(wakeup)) [[unlikely]] {
      return false;
    }

    return true;
  }

  auto begin() noexcept -> bool {
    static constexpr std::array request_payload{common::as_byte(0xF0)};
    static constexpr ECUMessage request{0x72, ECUMode::INIT, request_payload};
    static constexpr ECUMessage response{0x02, ECUMode::INIT};

    if (!send_message(request)) [[unlikely]] {
      return false;
    }

    ECUMessage answer{};

    if (!receive_message(answer)) [[unlikely]] {
      return false;
    }

    if (answer != response) [[unlikely]] {
      return false;
    }

    return true;
  }

  auto end() noexcept -> bool {
    static constexpr std::array payload{common::as_byte(0xF1)};
    static constexpr ECUMessage request{0x72, ECUMode::INIT, payload};
    static constexpr ECUMessage response{0x02, ECUMode::INIT};

    if (!send_message(request)) [[unlikely]] {
      return false;
    }

    ECUMessage answer{};

    if (!receive_message(answer)) [[unlikely]] {
      return false;
    }

    if (answer != response) [[unlikely]] {
      return false;
    }

    return true;
  }

  template <std::size_t PayloadSize>
  auto send_message(ECUMessage<PayloadSize> const& message) noexcept -> bool {
    static constexpr std::size_t packet_size{ECUMessage<PayloadSize>::total_size};

    if (!m_driver_uart.flush()) [[unlikely]] {
      return false;
    }

    if (int const written_bytes = m_driver_uart.write(message.to_array()); std::cmp_less(written_bytes, packet_size)) [[unlikely]] {
      return false;
    }

    std::array<std::uint8_t, packet_size> echo_packet{};

    if (!m_driver_uart.read(echo_packet, timeout_ms)) [[unlikely]] {
      return false;
    }

    auto const echo_message = ECUMessage<PayloadSize>{echo_packet};

    if (!echo_message.is_valid() || message != echo_message) [[unlikely]] {
      return false;
    }

    return true;
  }

  template <std::size_t PayloadSize>
  auto receive_message(ECUMessage<PayloadSize>& message) noexcept -> bool {
    static constexpr std::size_t packet_size{ECUMessage<PayloadSize>::total_size};

    std::array<std::uint8_t, packet_size> received_packet{};

    if (!m_driver_uart.read(received_packet, timeout_ms)) [[unlikely]] {
      return false;
    }

    auto const received_message = ECUMessage<PayloadSize>{received_packet};

    if (!received_message.is_valid()) [[unlikely]] {
      return false;
    }

    message = received_message;

    return true;
  }
};

}  // namespace device
