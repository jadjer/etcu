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

#include "common/convert.hpp"
#include "config/concepts.hpp"
#include "device/ecu/ecu_message.hpp"
#include "type/type.hpp"

namespace device {

struct EngineData {
  bool is_clutch;
  bool is_running;
  std::uint8_t speed;
  std::uint8_t ignition_advance;
  std::uint16_t rpm;
  std::uint16_t fuel_inject;
  std::float_t ect_temp;
  std::float_t iat_temp;
  std::float_t ect_voltage;
  std::float_t iat_voltage;
  std::float_t tps_percent;
  std::float_t tps_voltage;
  std::float_t map_voltage;
  std::float_t map_pressure;
  std::float_t battery_voltage;
};

template <class DriverUart, class DriverGPIO>
  requires concepts::UART<DriverUart> && concepts::GPIO<DriverGPIO>
class ECU {
  static constexpr std::uint16_t timeout_ms{100};

  DriverUart& m_driver_uart;
  DriverGPIO& m_driver_gpio;

  bool m_is_connected = false;

  EngineData m_engine_data{};

  template <std::size_t PayloadSize>
  auto send_message(ECUMessage<PayloadSize> const& message) noexcept -> bool {
    static constexpr std::size_t packet_size{ECUMessage<PayloadSize>::total_size};

    m_driver_uart.flush();

    if (int const written_bytes = m_driver_uart.write(message.to_array()); std::cmp_less(written_bytes, packet_size)) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    m_driver_uart.wait_send_done(timeout_ms);

    std::array<std::uint8_t, packet_size> echo_packet{};

    if (!m_driver_uart.read(echo_packet, timeout_ms)) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    auto const echo_message = ECUMessage<PayloadSize>{echo_packet};

    if (!echo_message.is_valid()) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    if (echo_message.checksum != echo_message.calculate_checksum()) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    if (message != echo_message) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    return true;
  }

  template <std::size_t PayloadSize>
  auto receive_message(ECUMessage<PayloadSize>& message) noexcept -> bool {
    static constexpr std::size_t packet_size{ECUMessage<PayloadSize>::total_size};

    std::array<std::uint8_t, packet_size> received_packet{};

    if (!m_driver_uart.read(received_packet, timeout_ms)) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    message = ECUMessage<PayloadSize>{received_packet};

    if (!message.is_valid()) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    if (message.checksum != message.calculate_checksum()) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    return true;
  }

  auto wake_up() noexcept -> bool {
    static constexpr ECUMessage<0> wakeup_message{0xFE, ECUMode::WAKE_UP};
    static constexpr ECUMessage<1> init_message{0x72, ECUMode::INIT, {0xF0}};
    static constexpr ECUMessage<0> init_answer_packet{0x02, ECUMode::INIT};

    m_driver_gpio.init();

    m_driver_gpio.disable();

    vTaskDelay(pdMS_TO_TICKS(70));

    m_driver_gpio.enable();

    vTaskDelay(pdMS_TO_TICKS(120));

    m_driver_uart.init();

    if (!send_message(wakeup_message)) [[unlikely]]
      return false;

    vTaskDelay(pdMS_TO_TICKS(200));

    if (!send_message(init_message)) [[unlikely]]
      return false;

    ECUMessage<0> answer_message{};

    if (!receive_message(answer_message)) [[unlikely]]
      return false;

    if (answer_message != init_answer_packet)
      return false;

    return true;
  }

  auto connect() noexcept -> bool {
    if (m_is_connected) [[likely]]
      return true;

    if (!wake_up()) [[unlikely]]
      return false;

    m_is_connected = true;

    return true;
  }

  auto updateTable11() noexcept -> bool {
    static constexpr std::uint8_t table{0x11};
    static constexpr std::uint8_t address{0x72};
    static constexpr std::uint8_t header_size{2};
    static constexpr std::uint8_t payload_size{0x17};
    static constexpr std::uint8_t payload_offset{0x0};
    static constexpr std::array payload{table, payload_offset, payload_size};
    static constexpr ECUMessage request{address, ECUMode::READ_RANGE, payload};

    if (!connect()) [[unlikely]]
      return false;

    if (!send_message(request)) [[unlikely]]
      return false;

    ECUMessage<header_size + payload_size> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    m_engine_data.rpm = (static_cast<std::uint16_t>(response.payload[2]) << 8) | response.payload[3];
    m_engine_data.tps_voltage = common::calculateValueDivide256(response.payload[4]);
    m_engine_data.tps_percent = common::calculateValueDivide16(response.payload[5]);
    m_engine_data.ect_voltage = common::calculateValueDivide256(response.payload[6]);
    m_engine_data.ect_temp = common::calculateValueMinus40(response.payload[7]);
    m_engine_data.iat_voltage = common::calculateValueDivide256(response.payload[8]);
    m_engine_data.iat_temp = common::calculateValueMinus40(response.payload[9]);
    m_engine_data.map_voltage = common::calculateValueDivide256(response.payload[10]);
    m_engine_data.map_pressure = response.payload[11];
    m_engine_data.battery_voltage = common::calculateValueDivide10(response.payload[14]);
    m_engine_data.speed = response.payload[15];
    m_engine_data.fuel_inject = (static_cast<std::uint16_t>(response.payload[16]) << 8) | response.payload[17];
    m_engine_data.ignition_advance = response.payload[18];

    return true;
  }

  auto updateTableD1() noexcept -> bool {
    static constexpr std::uint8_t table{0xD1};
    static constexpr std::uint8_t address{0x72};
    static constexpr std::uint8_t header_size{2};
    static constexpr std::uint8_t payload_size{0x06};
    static constexpr std::uint8_t payload_offset{0x0};
    static constexpr std::array payload{table, payload_offset, payload_size};
    static constexpr ECUMessage request{address, ECUMode::READ_RANGE, payload};

    if (!connect()) [[unlikely]]
      return false;

    if (!send_message(request)) [[unlikely]]
      return false;

    ECUMessage<header_size + payload_size> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    m_engine_data.is_clutch = response.payload[2];
    m_engine_data.is_running = response.payload[6];

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

    if (!updateTable11())
      return type::SystemError::ECUReadFault;
    if (!updateTableD1())
      return type::SystemError::ECUReadFault;

    return type::SystemError::None;
  }

  [[nodiscard]] auto get_telemetry(type::ECUTelemetry& telemetry) noexcept -> type::SystemError {
    if (!connect()) [[unlikely]]
      return type::SystemError::ECUInitFault;

    telemetry.is_connected = m_is_connected;
    telemetry.rpm = type::RPM{m_engine_data.rpm};
    telemetry.speed = type::Speed{m_engine_data.speed};
    telemetry.tps = type::Position{static_cast<int>(m_engine_data.tps_percent)};
    telemetry.is_started = false;
    telemetry.is_clutch_enabled = false;

    return type::SystemError::None;
  }
};

}  // namespace device
