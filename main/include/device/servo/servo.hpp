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

#include "common/atomic_container.hpp"
#include "common/calculate.hpp"
#include "common/convert.hpp"
#include "common/range.hpp"
#include "device/servo/register.hpp"
#include "device/servo/servo_message.hpp"
#include "device/servo/servo_protocol.hpp"
#include "type/error.hpp"
#include "type/type.hpp"

namespace device {

template <class Driver, class PowerEnable, std::uint8_t ServoId = 1>
  requires concepts::UART<Driver> && concepts::GPIO<PowerEnable> && (ServoId > 0) && (ServoId < 254)
class Servo {
  static constexpr std::uint8_t servo_id{ServoId};

  ServoProtocol<Driver, PowerEnable> m_protocol{};

  common::AtomicContainer<type::ServoCalibrationData> m_calibration_data{};

  [[nodiscard]] static constexpr auto parse_status_errors(std::uint8_t const instruction_or_status) noexcept -> type::SystemError {
    auto const current_status = static_cast<ServoError>(instruction_or_status);
    if (current_status == ServoError::None) {
      return type::SystemError::None;
    }

    auto system_error = type::SystemError::None;
    if (hasError(current_status, ServoError::Voltage)) {
      system_error = system_error | type::SystemError::ServoVoltageFailed;
    }
    if (hasError(current_status, ServoError::Encoder)) {
      system_error = system_error | type::SystemError::ServoEncoderFailed;
    }
    if (hasError(current_status, ServoError::Overheat)) {
      system_error = system_error | type::SystemError::ServoOverheat;
    }
    if (hasError(current_status, ServoError::Overload)) {
      system_error = system_error | type::SystemError::ServoOverload;
    }

    return system_error;
  }

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
      return type::SystemError::ServoInitFailed;
    }

    return type::SystemError::None;
  }

  auto set_calibration(type::ServoCalibrationData const& calibration_data) noexcept -> void { m_calibration_data.store(calibration_data); }

  [[nodiscard]] auto set_position(type::Position const target_position) noexcept -> type::SystemError {
    static constexpr type::Position position_min{type::Position::value_min};
    static constexpr type::Position position_max{type::Position::value_max};

    const auto [_, position] = m_calibration_data.load();

    type::ServoPosition const servo_position = common::map_range(target_position, position_min, position_max, position.min, position.max);

    std::array const params{
        common::as_byte(ServoRegister::TargetPosition),
        common::as_byte(servo_position.value),
        common::as_byte(servo_position.value >> 8),
    };

    if (ServoMessage const request{servo_id, ServoInstruction::Write, params}; !m_protocol.send_message(request)) [[unlikely]] {
      return type::SystemError::ServoWriteFailed;
    }

    ServoMessage response_message{};

    if (!m_protocol.receive_message(response_message)) {
      return type::SystemError::ServoReadFailed;
    }

    return parse_status_errors(response_message.instruction_or_status);
  }

  [[nodiscard]] auto get_telemetry(type::ServoTelemetry& telemetry) noexcept -> type::SystemError {
    static constexpr std::size_t payload_size{31};
    static constexpr std::array params{
        common::as_byte(ServoRegister::TorqueEnable),
        common::as_byte(payload_size),
    };
    static constexpr ServoMessage request{servo_id, ServoInstruction::Read, params};

    if (!m_protocol.send_message(request)) [[unlikely]] {
      return type::SystemError::ServoWriteFailed;
    }

    ServoMessage<payload_size> response{};

    if (!m_protocol.receive_message(response)) {
      return type::SystemError::ServoReadFailed;
    }

    telemetry.is_connected = true;
    telemetry.is_enabled = response.payload[0] != 0;
    telemetry.position = common::as_ulong(response.payload[17], response.payload[16]) & 0x7FFF;
    telemetry.voltage = common::calculateValueDivide10(response.payload[22]);
    telemetry.temperature = response.payload[23];
    telemetry.is_moved = response.payload[26] != 0;

    std::uint16_t const raw_current = common::as_ulong(response.payload[30], response.payload[29]) & 0x7FFF;
    telemetry.current = common::calculateValueMultiply10(raw_current);

    return parse_status_errors(response.instruction_or_status);
  }
};

}  // namespace device
