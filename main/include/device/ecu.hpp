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

#include <numeric>

#include "config/concepts.hpp"
#include "type/type.hpp"

namespace device {

template <std::size_t PayloadSize>
struct Message {
  static constexpr std::size_t header_size{3};
  static constexpr std::size_t checksum_size{1};
  static constexpr std::size_t total_size{header_size + PayloadSize + checksum_size};

  std::uint8_t address;
  std::uint8_t length;
  std::uint8_t mode;
  std::array<std::uint8_t, PayloadSize> payload;
  std::uint8_t checksum;

  [[nodiscard]] auto calculate_checksum() const noexcept -> std::uint8_t {
    std::uint32_t sum = address + length + mode;
    if constexpr (PayloadSize > 0) {
      sum = std::accumulate(payload.begin(), payload.end(), sum);
    }
    return static_cast<std::uint8_t>(0U - sum);
  }

  [[nodiscard]] auto to_array() const noexcept -> std::array<std::uint8_t, total_size> {
    std::array<std::uint8_t, total_size> bytes{};

    bytes[0] = address;
    bytes[1] = length;
    bytes[2] = mode;

    if constexpr (PayloadSize > 0) {
      std::copy(payload.begin(), payload.end(), bytes.begin() + header_size);
    }

    bytes[total_size - 1] = calculate_checksum();

    return bytes;
  }

  static auto from_array(std::array<std::uint8_t, total_size> const& bytes) noexcept -> Message {
    Message msg{};

    msg.address = bytes[0];
    msg.length = bytes[1];
    msg.mode = bytes[2];

    if constexpr (PayloadSize > 0) {
      std::copy(bytes.begin() + header_size, bytes.begin() + (header_size + PayloadSize), msg.payload.begin());
    }

    msg.checksum = bytes[total_size - 1];

    return msg;
  }
};

struct EngineData {
  std::uint16_t rpm;
  std::uint16_t fuelInject;
  std::uint8_t ignitionAdvance;
  std::uint16_t tpsPercent;
  std::uint16_t tpsVolts;
  std::uint8_t ectTemp;
  std::uint16_t ectVolts;
  std::uint8_t iatTemp;
  std::uint16_t iatVolts;
  std::uint8_t mapPressure;
  std::uint16_t mapVolts;
  std::uint16_t batteryVolts;
  std::uint8_t speed;
  std::uint8_t state;
};

template <class DriverUart, class DriverGPIO>
  requires concepts::UART<DriverUart> && concepts::GPIO<DriverGPIO>
class ECU {
  DriverUart& m_driver_uart;
  DriverGPIO& m_driver_gpio;

  bool m_is_connected = false;

  EngineData m_engine_data{};

  template <std::size_t PayloadSize>
  auto send_message(Message<PayloadSize> const& message) noexcept -> bool {
    if (!m_driver_uart.flush()) [[unlikely]] {
      ESP_LOGE("ECU", "Flush");
      return false;
    }

    if (!m_driver_uart.write(message.to_array())) [[unlikely]] {
      ESP_LOGE("ECU", "Write message");
      return false;
    }

    return true;
  }

  template <std::size_t PayloadSize>
  auto receive_message(Message<PayloadSize>& message) noexcept -> bool {
    static constexpr std::uint16_t timeout_ms{1000};
    static constexpr std::size_t package_size{Message<PayloadSize>::total_size};

    std::array<std::uint8_t, package_size> packet{};

    if (!m_driver_uart.read(packet, timeout_ms)) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    message = Message<PayloadSize>::from_array(packet);

    std::uint8_t const checksum_response = packet.back();
    std::uint8_t const checksum_calculated = message.calculate_checksum();

    if (checksum_response != checksum_calculated) [[unlikely]] {
      return false;
    }

    return true;
  }

  auto wake_up() noexcept -> bool {
    static constexpr Message<0> wakeup_message{ 0xFE,  0x04,  0xFF,  {},  0xFF};

    m_driver_gpio.init();

    m_driver_gpio.disable();
    vTaskDelay(pdMS_TO_TICKS(70));

    m_driver_gpio.enable();
    vTaskDelay(pdMS_TO_TICKS(120));

    m_driver_uart.init();

    return send_message(wakeup_message);
  }

  auto initialize() noexcept -> bool {
    static constexpr Message<1> init_message{0x72, 0x05, 0x00, {0xF0}, 0x99};
    static constexpr Message<0> init_answer_packet{0x02, 0x04, 0x00, {}, 0xFA};

    send_message(init_message);

    Message<0> answer_message{};
    receive_message(answer_message);

    return answer_message.to_array() == init_answer_packet.to_array();
  }

  auto connect() noexcept -> bool {
    if (m_is_connected) [[likely]]
      return true;

    if (!wake_up()) [[unlikely]]
      return false;

    if (!initialize()) [[unlikely]]
      return false;

    m_is_connected = true;

    return true;
  }

  auto updateTable0() noexcept -> bool {
    static constexpr Message<1> request{0x72, 5, 0x71, {}, 0x18};
    if (!send_message(request)) [[unlikely]]
      return false;

    Message<11> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    return true;
  }

  auto updateTable10() noexcept -> bool {
    static constexpr Message<1> request{0x72, 5, 0x71, {0x10}, 0x8};
    if (!send_message(request)) [[unlikely]]
      return false;

    Message<18> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    return true;
  }

  auto updateTable11() noexcept -> bool {
    static constexpr Message<1> request{0x72, 5, 0x71, {0x11}, 0x7};
    if (!send_message(request)) [[unlikely]]
      return false;

    Message<21> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    return true;
  }

  auto updateTable20() noexcept -> bool {
    static constexpr Message<1> request{0x72, 5, 0x71, {0x20}, 0xF8};
    if (!send_message(request)) [[unlikely]]
      return false;

    Message<4> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    return true;
  }

  auto updateTable21() noexcept -> bool {
    static constexpr Message<1> request{0x72, 5, 0x71, {0x21}, 0xF7};
    if (!send_message(request)) [[unlikely]]
      return false;

    Message<7> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    return true;
  }

  auto updateTableD0() noexcept -> bool {
    static constexpr Message<1> request{0x72, 5, 0x71, {0xD0}, 0x48};
    if (!send_message(request)) [[unlikely]]
      return false;

    Message<15> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    return true;
  }

  auto updateTableD1() noexcept -> bool {
    static constexpr Message<1> request{0x72, 5, 0x71, {0xD1}, 0x47};
    if (!send_message(request)) [[unlikely]]
      return false;

    Message<7> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    return true;
  }

 public:
  constexpr explicit ECU(DriverUart& driver_uart, DriverGPIO& driver_gpio) noexcept : m_driver_uart{driver_uart}, m_driver_gpio{driver_gpio} {}

  constexpr ECU() noexcept = delete;

  ECU(ECU const&) noexcept = delete;
  auto operator=(ECU const&) noexcept -> ECU& = delete;

  ECU(ECU&&) noexcept = delete;
  auto operator=(ECU&&) noexcept -> ECU& = delete;

  constexpr ~ECU() noexcept = default;

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_driver_uart.init()) [[unlikely]]
      return type::SystemError::ECUInitFault;

    if (!m_driver_gpio.init()) [[unlikely]]
      return type::SystemError::ECUInitFault;

    if (!connect()) [[unlikely]]
      return type::SystemError::ECUInitFault;

    return type::SystemError::None;
  }

  [[nodiscard]] auto update() noexcept -> type::SystemError {
    if (!connect()) [[unlikely]]
      return type::SystemError::ECUInitFault;

    updateTable0();
    updateTable10();
    updateTable11();
    updateTable20();
    updateTable21();
    updateTableD0();
    updateTableD1();

    return type::SystemError::None;
  }

  [[nodiscard]] auto get_telemetry(type::ECUTelemetry& telemetry) noexcept -> type::SystemError {
    if (!connect()) [[unlikely]]
      return type::SystemError::ECUInitFault;

    telemetry.is_connected = m_is_connected;
    telemetry.rpm = type::RPM{m_engine_data.rpm};
    telemetry.speed = type::Speed{m_engine_data.speed};
    telemetry.tps = type::Position{m_engine_data.tpsPercent};
    telemetry.is_started = false;
    telemetry.is_clutch_enabled = false;

    return type::SystemError::None;
  }
};

}  // namespace device
