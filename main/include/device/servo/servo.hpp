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
#include "type/type.hpp"

namespace device {

template <class Driver, class PowerEnable, std::uint8_t ServoId = 1>
  requires concepts::UART<Driver> && concepts::GPIO<PowerEnable> && (ServoId > 0) && (ServoId < 254)
class Servo {
  static constexpr std::uint8_t servo_id{ServoId};

  type::ServoCalibrationData m_calibration_data{
      .position_minimal{600},
      .position_maximal{1250},
  };

  Driver& m_driver_uart;
  PowerEnable& m_driver_power;

  template <std::size_t ParamSize>
  constexpr auto send_packet(ServoInstruction const instruction, std::array<std::uint8_t, ParamSize> const& parameters) const noexcept -> void {
    ServoMessage<ParamSize> const message{servo_id, instruction, parameters};

    m_driver_uart.flush();
    m_driver_uart.write(message.to_array());
  }

  template <std::size_t PayloadSize>
  constexpr auto receive_message(ServoMessage<PayloadSize>& message) noexcept -> bool {
    static constexpr std::uint16_t timeout_ms{30};
    static constexpr std::size_t total_package_size = ServoMessage<PayloadSize>::total_size;

    std::array<std::uint8_t, total_package_size> response_bytes{};

    if (!m_driver_uart.read(response_bytes, timeout_ms)) [[unlikely]]
      return false;

    auto const response_message = ServoMessage<PayloadSize>{servo_id, response_bytes};

    if (!response_message.is_valid()) [[unlikely]]
      return false;

    message = response_message;

    return true;
  }

 public:
  constexpr explicit Servo(Driver& driver_uart, PowerEnable& driver_power) noexcept : m_driver_uart(driver_uart), m_driver_power(driver_power) {}

  constexpr Servo() noexcept = delete;

  Servo(Servo const&) noexcept = delete;
  auto operator=(Servo const&) noexcept -> Servo& = delete;

  Servo(Servo&&) noexcept = delete;
  auto operator=(Servo&&) noexcept -> Servo& = delete;

  constexpr ~Servo() noexcept = default;

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_driver_uart.init()) [[unlikely]]
      return type::SystemError::ServoInitError;

    if (!m_driver_power.init()) [[unlikely]]
      return type::SystemError::ServoInitError;

    if (!m_driver_power.enable()) [[unlikely]]
      return type::SystemError::ServoPowerFail;

    return type::SystemError::None;
  }

  auto set_position(type::Position const target_position) noexcept -> bool {
    static constexpr type::Position position_min{type::Position::value_min};
    static constexpr type::Position position_max{type::Position::value_max};

    type::ServoPosition const servo_position =
        common::map_range(target_position, position_min, position_max, m_calibration_data.position_minimal, m_calibration_data.position_maximal);

    std::array const params{
        common::as_byte(ServoRegister::TargetPosition),
        common::as_byte(servo_position.value),
        common::as_byte(servo_position.value >> 8),
    };
    send_packet(ServoInstruction::InstWrite, params);

    if (ServoMessage<0> message{}; !receive_message(message))
      return false;

    return true;
  }

  auto get_telemetry(type::ServoTelemetry& telemetry) noexcept -> bool {
    static constexpr std::size_t payload_size{31};

    static constexpr std::array params{
        common::as_byte(ServoRegister::TorqueEnable),
        common::as_byte(payload_size),
    };
    send_packet(ServoInstruction::InstRead, params);

    ServoMessage<payload_size> message{};

    if (!receive_message(message))
      return false;

    telemetry.is_connected = true;
    telemetry.is_enabled = message.payload[0];
    telemetry.position = common::as_ulong(message.payload[17], message.payload[16]) & 0x7FFF;
    telemetry.voltage = common::calculateValueDivide10(message.payload[22]);
    telemetry.temperature = message.payload[23];
    telemetry.is_moved = message.payload[26];
    telemetry.current = common::calculateValueMultiply10(common::as_ulong(message.payload[30], message.payload[29]) & 0x7FFF);

    return true;
  }
};

}  // namespace device
