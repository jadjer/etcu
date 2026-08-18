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
#include <utility>
#include "common/map_range.hpp"
#include "type.hpp"

namespace device {

template <class Driver, class PowerEnable, type::ServoId const servoId = 1>
  requires concepts::UART<Driver> && concepts::GPIO<PowerEnable>

class Servo {
  type::ServoCalibrationData const m_calibration_data{
      .position_minimal = 600,
      .position_maximal = 1250,
  };

  Driver& m_uart_driver;
  PowerEnable& m_power_enable;

  template <type::Size packetSize>
    requires(packetSize >= 6)
  [[nodiscard]] auto calculate_checksum_for_packet(std::array<type::Byte, packetSize> const bytes) const noexcept -> type::Byte {
    std::size_t constexpr start_index = 2;
    std::size_t constexpr end_index = packetSize - 1;

    std::uint32_t const sum = std::accumulate(bytes.begin() + start_index, bytes.begin() + end_index, 0U);
    return static_cast<type::Byte>(~(sum & 0xFF));
  }

  template <std::size_t paramSize>
  auto send_packet(type::ServoInstruction const instruction, std::array<type::Byte, paramSize> const& parameters) const noexcept -> void {
    std::size_t constexpr packet_size = paramSize + 6;
    std::array<type::Byte, packet_size> packet;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servoId;
    packet[3] = paramSize + 2;
    packet[4] = +instruction;

    if constexpr (paramSize > 0) {
      for (std::size_t i = 0; i < paramSize; ++i) {
        packet[5 + i] = parameters[i];
      }
    }

    packet[5 + paramSize] = calculate_checksum_for_packet(packet);

    std::ignore = m_uart_driver.flush();
    std::ignore = m_uart_driver.write(packet);
  }

  template <std::size_t paramSize>
  auto receive_packet(std::array<type::Byte, paramSize>& payload) noexcept -> bool {
    if (servoId == 0xFE)
      return false;

    std::size_t constexpr packetSize = paramSize + 6;

    std::array<type::Byte, packetSize> response;

    if (int const read_bytes = m_uart_driver.read(response, 30); std::cmp_less(read_bytes, packetSize)) {
      return false;
    }

    if (response[0] != 0xFF || response[1] != 0xFF || response[2] != servoId) {
      return false;
    }

    type::Byte const calculated_check_sum = calculate_checksum_for_packet(response);

    if (calculated_check_sum != response[packetSize - 1]) {
      return false;
    }

    if constexpr (paramSize > 0) {
      for (std::size_t i = 0; i < paramSize; ++i) {
        payload[i] = static_cast<type::Byte>(response[5 + i]);
      }
    }

    return true;
  }

 public:
  explicit Servo(Driver& uart_driver, PowerEnable& power_enable) noexcept : m_uart_driver(uart_driver), m_power_enable(power_enable) {}

  auto init() noexcept -> type::SystemError {
    if (!m_power_enable.init())
      return type::SystemError::ServoInitError;
    if (!m_uart_driver.init())
      return type::SystemError::ServoInitError;

    return type::SystemError::None;
  }

  auto set_position(type::Position const target_position) noexcept -> void {
    type::Position constexpr minimalPosition = 0;
    type::Position constexpr maximalPosition = 100;

    type::ServoPosition const servo_position =
        commons::map_range(target_position, minimalPosition, maximalPosition, m_calibration_data.position_minimal, m_calibration_data.position_maximal);

    auto const servo_position_value = servo_position.get();

    std::array<type::Byte, 7> params{};
    params[0] = +type::ServoRegister::RegTargetPosition;
    params[1] = servo_position_value & 0xFF;
    params[2] = (servo_position_value >> 8) & 0xFF;
    params[3] = 0 & 0xFF;
    params[4] = (0 >> 8) & 0xFF;
    params[5] = 0 & 0xFF;
    params[6] = (0 >> 8) & 0xFF;
    send_packet(type::ServoInstruction::InstWrite, params);

    std::array<type::Byte, 0> payload;
    receive_packet(payload);
  }

  auto get_telemetry(type::ServoTelemetry& telemetry) noexcept -> bool {
    std::array<type::Byte, 2> constexpr params{+type::ServoRegister::RegPresentPosition, 15};
    send_packet(type::ServoInstruction::InstRead, params);

    if (std::array<type::Byte, 15> payload{}; receive_packet(payload)) {
      telemetry.is_connected = true;
      telemetry.position = ((payload[1] << 8) | payload[0]) & 0x7FFF;
      telemetry.speed = ((payload[3] << 8) | payload[2]) & 0x7FFF;
      telemetry.load = static_cast<type::Load>(((payload[5] << 8) | payload[4]) & 0x03FF);
      telemetry.voltage = payload[6];
      telemetry.temperature = payload[7];
      telemetry.is_moved = payload[10];
      telemetry.current = ((payload[14] << 8) | payload[13]) & 0x7FFF;

      return true;
    }

    return false;
  }

  auto set_mode(type::ServoMode const mode) noexcept -> void {
    std::array const params{+type::ServoRegister::RegMode, +mode};
    send_packet(type::ServoInstruction::InstWrite, params);

    std::array<type::Byte, 0> payload;
    receive_packet(payload);
  }

  auto set_speed(std::int16_t const speed) noexcept -> void {
    auto const abs_speed = static_cast<int16_t>(std::abs(speed));

    std::uint16_t reg_value = abs_speed;

    if (speed < 0) {
      reg_value |= 0x8000;
    }

    auto const low_byte = static_cast<type::Byte>(reg_value & 0xFF);
    auto const high_byte = static_cast<type::Byte>((reg_value >> 8) & 0xFF);

    std::array const params{+type::ServoRegister::RegTargetSpeed, low_byte, high_byte};
    send_packet(type::ServoInstruction::InstWrite, params);

    std::array<type::Byte, 0> payload;
    receive_packet(payload);
  }

  auto read_current(type::Current& current) noexcept -> bool {
    std::array<type::Byte, 2> constexpr params{+type::ServoRegister::RegPresentCurrent, 2};
    send_packet(type::ServoInstruction::InstRead, params);

    if (std::array<type::Byte, 2> payload{}; receive_packet(payload)) {
      current = ((payload[1] << 8) | payload[0]) & 0x7FFF;

      return true;
    }

    return false;
  }

  auto read_position(type::ServoPosition& position) noexcept -> bool {
    std::array<type::Byte, 2> constexpr params{+type::ServoRegister::RegPresentPosition, 2};
    send_packet(type::ServoInstruction::InstRead, params);

    if (std::array<type::Byte, 2> payload{}; receive_packet(payload)) {
      position = payload[0] | (payload[1] << 8);

      return true;
    }

    return false;
  }
};

}  // namespace device
