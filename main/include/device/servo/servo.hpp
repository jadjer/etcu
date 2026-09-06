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

#include "common/calculate.hpp"
#include "common/convert.hpp"
#include "common/map_range.hpp"
#include "device/servo/register.hpp"
#include "device/servo/servo_message.hpp"
#include "device/servo/servo_protocol.hpp"
#include "type/error.hpp"
#include "type/type.hpp"

namespace device {

template <class Driver, class PowerEnable, std::uint8_t ServoId = 1>
  requires concepts::UART<Driver> && concepts::GPIO<PowerEnable> && (ServoId > 0) && (ServoId < 254)
class Servo {
  type::ServoCalibrationData m_calibration_data{};

  ServoProtocol<Driver, PowerEnable, ServoId> m_protocol;

 public:
  constexpr explicit Servo(Driver& driver_uart, PowerEnable& driver_power) noexcept : m_protocol(driver_uart, driver_power) {}

  constexpr Servo() noexcept = delete;

  Servo(Servo const&) noexcept = delete;
  auto operator=(Servo const&) noexcept -> Servo& = delete;

  Servo(Servo&&) noexcept = delete;
  auto operator=(Servo&&) noexcept -> Servo& = delete;

  constexpr ~Servo() noexcept = default;

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_protocol.init()) [[unlikely]] {
      return type::SystemError::ServoInitError;
    }

    return type::SystemError::None;
  }

  auto set_calibration(type::ServoCalibrationData const& calibration_data) noexcept -> void { m_calibration_data = calibration_data; }

  [[nodiscard]] auto set_position(type::Position const target_position) noexcept -> type::SystemError {
    static constexpr type::Position position_min{type::Position::value_min};
    static constexpr type::Position position_max{type::Position::value_max};

    type::ServoPosition const servo_position =
        common::map_range(target_position, position_min, position_max, m_calibration_data.position_minimal, m_calibration_data.position_maximal);

    std::array const params{
        common::as_byte(ServoRegister::TargetPosition),
        common::as_byte(servo_position.value),
        common::as_byte(servo_position.value >> 8),
    };

    if (!m_protocol.send_packet(ServoInstruction::InstWrite, params)) [[unlikely]] {
      return type::SystemError::ServoWriteError;
    }

    if (ServoMessage response_message{}; !m_protocol.receive_packet(response_message)) {
      return type::SystemError::ServoReadError;
    }

    return type::SystemError::None;
  }

  [[nodiscard]] auto get_telemetry(type::ServoTelemetry& telemetry) noexcept -> type::SystemError {
    static constexpr std::size_t payload_size{31};

    static constexpr std::array params{
        common::as_byte(ServoRegister::TorqueEnable),
        common::as_byte(payload_size),
    };

    m_protocol.send_packet(ServoInstruction::InstRead, params);

    ServoMessage<payload_size> response_message{};

    if (!m_protocol.receive_packet(response_message)) {
      telemetry.is_connected = false;
      return type::SystemError::ServoReadError;
    }

    telemetry.is_connected = true;
    telemetry.is_enabled = (response_message.payload[0] != 0);
    telemetry.position = common::as_ulong(response_message.payload[17], response_message.payload[16]) & 0x7FFF;
    telemetry.voltage = common::calculateValueDivide10(response_message.payload[22]);
    telemetry.temperature = response_message.payload[23];
    telemetry.is_moved = (response_message.payload[26] != 0);

    std::uint32_t const raw_current = common::as_ulong(response_message.payload[30], response_message.payload[29]) & 0x7FFF;
    telemetry.current = common::calculateValueMultiply10(raw_current);

    return type::SystemError::None;
  }
};

}  // namespace device
