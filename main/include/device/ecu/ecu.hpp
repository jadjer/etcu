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

#include <array>
#include <span>
#include "common/calculate.hpp"
#include "config/concepts.hpp"
#include "device/ecu/ecu_message.hpp"
#include "device/ecu/ecu_protocol.hpp"
#include "type/type.hpp"

namespace device {

enum class EngineState : std::uint8_t {
  GEAR_ON = 0x00,
  NEUTRAL = 0x01,
  SIDE_STAND_AND_GEAR_ON = 0x02,
  SIDE_STAND = 0x03,
};

struct EngineData {
  bool is_running{false};
  EngineState state{EngineState::NEUTRAL};
  std::uint8_t speed{0};
  std::uint8_t ect_temp{0};
  std::uint8_t iat_temp{0};
  std::uint8_t ect_voltage{0};
  std::uint8_t iat_voltage{0};
  std::uint8_t tps_percent{0};
  std::uint8_t tps_voltage{0};
  std::uint8_t map_voltage{0};
  std::uint8_t map_pressure{99};
  std::uint8_t battery_voltage{0};
  std::uint8_t ignition_advance{0};
  std::uint16_t rpm{0};
  std::uint16_t fuel_inject{0};
};

template <class DriverUart, class DriverGPIO>
  requires concepts::UART<DriverUart> && concepts::GPIO<DriverGPIO>
class ECU {
  static constexpr std::size_t header_size{2};
  static constexpr std::size_t payload_size{17};
  static constexpr std::array m_supported_tables{0x10, 0x11, 0xD1};

  ECUProtocol<DriverUart, DriverGPIO> m_protocol;

  bool m_is_connected = false;
  EngineData m_engine_data{};
  std::span<std::uint8_t const> m_active_tables{};
  std::array<std::uint8_t, m_supported_tables.size()> m_active_tables_buffer{};

  [[nodiscard]] auto connect() noexcept -> bool {
    if (m_is_connected) [[likely]]
      return true;

    if (!m_protocol.wakeup()) [[unlikely]] {
      return false;
    }

    m_is_connected = true;

    detect_tables();

    return true;
  }

  auto detect_tables() noexcept -> void {
    static constexpr std::uint8_t address{0x72};
    static constexpr std::uint8_t probe_size{0x01};
    static constexpr std::uint8_t probe_offset{0x00};

    std::size_t found_count = 0;

    for (std::uint8_t const table : m_supported_tables) {
      std::array const payload{table, probe_offset, probe_size};

      if (ECUMessage const request{address, ECUMode::READ_RANGE, payload}; !m_protocol.send_message(request)) {
        continue;
      }

      if (ECUMessage<header_size + probe_size> response{}; m_protocol.receive_message(response)) {
        m_active_tables_buffer[found_count++] = table;
      }
    }

    m_active_tables = std::span<std::uint8_t const>{m_active_tables_buffer.data(), found_count};
  }

  auto update_active_tables() noexcept -> bool {
    static constexpr std::uint8_t address{0x72};
    static constexpr std::uint8_t payload_offset{0};

    for (std::uint8_t const table : m_active_tables) {
      std::array const payload{table, payload_offset, common::as_byte(payload_size)};

      if (ECUMessage const request{address, ECUMode::READ_RANGE, payload}; !m_protocol.send_message(request)) [[unlikely]] {
        return false;
      }

      ECUMessage<header_size + payload_size> response{};

      if (!m_protocol.receive_message(response)) [[unlikely]] {
        return false;
      }

      parse_table_data(table, response.payload);
    }

    return true;
  }

  auto parse_table_data(std::uint8_t const table, std::array<std::uint8_t, header_size + payload_size> const& payload) noexcept -> void {
    switch (table) {
      case 0x10:
      case 0x11:
        m_engine_data.rpm = common::as_ulong(payload[header_size + 0], payload[header_size + 1]);
        m_engine_data.tps_voltage = common::calculateValueDivide256(payload[header_size + 2]);
        m_engine_data.tps_percent = common::calculateValueDivide16(payload[header_size + 3]);
        m_engine_data.ect_voltage = common::calculateValueDivide256(payload[header_size + 4]);
        m_engine_data.ect_temp = common::calculateValueMinus40(payload[header_size + 5]);
        m_engine_data.iat_voltage = common::calculateValueDivide256(payload[header_size + 6]);
        m_engine_data.iat_temp = common::calculateValueMinus40(payload[header_size + 7]);
        m_engine_data.map_voltage = common::calculateValueDivide256(payload[header_size + 8]);
        m_engine_data.map_pressure = payload[header_size + 9];
        m_engine_data.battery_voltage = common::calculateValueDivide10(payload[header_size + 12]);
        m_engine_data.speed = payload[header_size + 13];
        m_engine_data.fuel_inject = common::as_ulong(payload[header_size + 14], payload[header_size + 15]);
        m_engine_data.ignition_advance = payload[header_size + 16];
        break;

      case 0xD1:
        m_engine_data.state = static_cast<EngineState>(payload[header_size + 0]);
        m_engine_data.is_running = (payload[header_size + 4] != 0);
        break;

      default:
        break;
    }
  }

 public:
  constexpr explicit ECU(DriverUart& driver_uart, DriverGPIO& driver_gpio) noexcept : m_protocol(driver_uart, driver_gpio) {}

  constexpr ECU() noexcept = delete;

  ECU(ECU const&) noexcept = delete;
  auto operator=(ECU const&) noexcept -> ECU& = delete;

  ECU(ECU&&) noexcept = delete;
  auto operator=(ECU&&) noexcept -> ECU& = delete;

  constexpr ~ECU() noexcept = default;

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_protocol.init()) [[unlikely]] {
      return type::SystemError::ECUInitFault;
    }

    m_is_connected = false;

    return type::SystemError::None;
  }

  [[nodiscard]] auto update() noexcept -> type::SystemError {
    if (!connect()) [[unlikely]] {
      return type::SystemError::None;
    }

    if (!update_active_tables()) [[unlikely]] {
      m_is_connected = false;
      return type::SystemError::ECUReadFault;
    }

    return type::SystemError::None;
  }

  [[nodiscard]] auto get_telemetry(type::ECUTelemetry& telemetry) const noexcept -> type::SystemError {
    telemetry.is_connected = m_is_connected;

    if (!m_is_connected) [[unlikely]] {
      telemetry.is_started = false;
      telemetry.is_neutral = true;

      telemetry.rpm = type::RPM{0};
      telemetry.battery = type::Volt{0};
      telemetry.speed = type::Speed{0};
      telemetry.map = type::Pressure{0};
      telemetry.tps = type::Position{0};
      telemetry.air = type::Temperature{0};
      telemetry.coolant = type::Temperature{0};

      return type::SystemError::None;
    }

    telemetry.is_started = m_engine_data.is_running;
    telemetry.is_neutral = (m_engine_data.state != EngineState::GEAR_ON);

    telemetry.rpm = type::RPM{m_engine_data.rpm};
    telemetry.battery = type::Volt{m_engine_data.battery_voltage};
    telemetry.speed = type::Speed{m_engine_data.speed};
    telemetry.map = type::Pressure{m_engine_data.map_pressure};
    telemetry.tps = type::Position{static_cast<std::int32_t>(std::roundf(m_engine_data.tps_percent))};
    telemetry.air = type::Temperature{static_cast<std::int32_t>(m_engine_data.iat_temp)};
    telemetry.coolant = type::Temperature{static_cast<std::int32_t>(m_engine_data.ect_temp)};

    return type::SystemError::None;
  }
};

}  // namespace device
