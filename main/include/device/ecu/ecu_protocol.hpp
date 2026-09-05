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

  [[nodiscard]] auto init() noexcept -> bool { return m_driver_uart.init() && m_driver_gpio.init(); }

  [[nodiscard]] auto wakeup() noexcept -> bool {
    static constexpr ECUMessage wakeup_message{0xFE, ECUMode::WAKE_UP};
    static constexpr std::array init_payload{common::as_byte(0xF0)};
    static constexpr ECUMessage init_message{0x72, ECUMode::INIT, init_payload};
    static constexpr ECUMessage init_answer_packet{0x02, ECUMode::INIT};

    if (!m_driver_gpio.disable()) [[unlikely]] {
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(70));

    if (!m_driver_gpio.enable()) [[unlikely]] {
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(120));

    if (!m_driver_uart.init()) [[unlikely]] {
      return false;
    }

    if (!send_message(wakeup_message)) [[unlikely]] {
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(200));

    if (!send_message(init_message)) [[unlikely]] {
      return false;
    }

    ECUMessage answer_message{};

    if (!receive_message(answer_message)) [[unlikely]] {
      return false;
    }

    return answer_message == init_answer_packet;
  }

  template <std::size_t PayloadSize>
  [[nodiscard]] auto send_message(ECUMessage<PayloadSize> const& message) noexcept -> bool {
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
  [[nodiscard]] auto receive_message(ECUMessage<PayloadSize>& message) noexcept -> bool {
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
