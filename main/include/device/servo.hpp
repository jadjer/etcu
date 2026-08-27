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
#include "type/type.hpp"

namespace device {

enum class ServoRegister : std::uint8_t {
  // =========================================================================
  // ЗОНА EEPROM (Энергонезависимая. Запись доступна только при RegLockSign = 0)
  // =========================================================================
  FormwareMajorVersion = 0x00,
  FormwareMinorVersion = 0x01,
  ServoMajorVersion = 0x03,
  ServoMinorVersion = 0x04,
  Id = 0x05,
  BaudRate = 0x06,
  ReturnDelayTime = 0x07,
  ResponseStatusLevel = 0x08,
  MinAngleLimit = 0x09,
  MaxAngleLimit = 0x0B,
  MaxTemperatureLimit = 0x0D,
  MaxInputVoltageLimit = 0x0E,
  MinInputVoltageLimit = 0x0F,
  MaxTorqueLimit = 0x10,
  Phase = 0x12,
  UnloadCondition = 0x13,
  LedAlarmCondition = 0x14,
  ComplianceP = 0x15,
  ComplianceD = 0x16,
  ComplianceI = 0x17,
  MinStartingForce = 0x18,
  CWDeadZone = 0x1A,
  CCWDeadZone = 0x1B,
  ProtectionCurrent = 0x1C,
  AngleResolution = 0x1E,
  PositionCorrection = 0x1F,
  OperationMode = 0x21,
  ProtectionTorque = 0x22,
  ProtectionTime = 0x23,
  OverloadTorque = 0x24,
  SpeedCloseLoopP = 0x25,
  OvercurrentProtectionTime = 0x26,
  VelocityCloseLoopI = 0x27,

  // =========================================================================
  // ЗОНА SRAM (Оперативная. Сбрасывается в дефолт при выключении питания)
  // =========================================================================
  TorqueEnable = 0x28,
  Acceleration = 0x29,
  TargetPosition = 0x2A,
  OperationTime = 0x2C,
  OperationSpeed = 0x2E,
  TorqueLimit = 0x30,

  LockFlag = 0x37,
  CurrentPosition = 0x38,
  CurrentSpeed = 0x3A,
  CurrentLoad = 0x3C,
  CurrentVoltage = 0x3E,
  CurrentTemperature = 0x3F,
  AsynchronousWriteFlag = 0x40,
  ServoStatus = 0x41,
  MoveFlag = 0x42,

  CurrentCurrent = 0x45,
};

enum class ServoInstruction : std::uint8_t {
  InstPing = 0x01,
  InstRead = 0x02,
  InstWrite = 0x03,
  InstRegWrite = 0x04,
  InstAction = 0x05,
  InstReset = 0x06,
  InstSyncWrite = 0x83
};

enum class ServoMode : std::uint8_t {
  PositionMode = 0x00,
  WheelMode = 0x01,
  PwmMode = 0x02,
  StepMode = 0x03,
};

template <typename T>
  requires std::is_enum_v<T> || std::convertible_to<T, std::uint8_t>
[[nodiscard]] constexpr auto as_byte(T const& value) noexcept -> std::uint8_t {
  return static_cast<std::uint8_t>(value) & 0xFF;
}

template <class Driver, class PowerEnable, std::uint8_t const ServoId = 1>
  requires concepts::UART<Driver> && concepts::GPIO<PowerEnable>

class Servo {
  static constexpr std::uint8_t servo_id{ServoId};

  type::ServoCalibrationData m_calibration_data{
      .position_minimal{600},
      .position_maximal{1250},
  };

  Driver& m_driver_uart;
  PowerEnable& m_driver_power;

  template <std::size_t PacketSize>
    requires(PacketSize >= 6)
  [[nodiscard]] auto calculate_checksum_for_packet(std::array<std::uint8_t, PacketSize> const& bytes) const noexcept -> std::uint8_t {
    static constexpr std::size_t start_index{2};
    static constexpr std::size_t end_index{PacketSize - 1};

    std::uint32_t const sum = std::accumulate(bytes.begin() + start_index, bytes.begin() + end_index, 0U);

    return static_cast<std::uint8_t>(~sum);
  }

  template <std::size_t ParamSize>
  auto send_packet(ServoInstruction const instruction, std::array<std::uint8_t, ParamSize> const& parameters) const noexcept -> void {
    static constexpr std::size_t param_size = ParamSize;
    static constexpr std::size_t packet_size = param_size + 6;

    std::array<std::uint8_t, packet_size> packet{};

    packet[0] = as_byte(0xFF);
    packet[1] = as_byte(0xFF);
    packet[2] = as_byte(servo_id);
    packet[3] = as_byte(param_size + 2);
    packet[4] = as_byte(instruction);

    if constexpr (param_size > 0)
      std::copy(parameters.begin(), parameters.end(), packet.begin() + 5);

    packet[5 + param_size] = calculate_checksum_for_packet(packet);

    std::ignore = m_driver_uart.flush();
    std::ignore = m_driver_uart.template write<packet_size>(packet);
  }

  template <std::size_t PayloadSize>
  [[nodiscard]] auto receive_packet(std::array<std::uint8_t, PayloadSize>& payload) noexcept -> bool {
    static constexpr std::size_t payload_size{PayloadSize};
    static constexpr std::size_t package_size{payload_size + 6};

    if (servo_id == 0xFE) [[unlikely]]
      return false;

    std::array<std::uint8_t, package_size> response;

    int const read_bytes = m_driver_uart.template read<package_size>(response, 30);
    if (std::cmp_less(read_bytes, package_size)) [[unlikely]]
      return false;

    if (response[0] != 0xFF || response[1] != 0xFF || response[2] != servo_id) [[unlikely]]
      return false;

    if (response[4] != 0x00) [[unlikely]]
      return false;

    if (calculate_checksum_for_packet(response) != response[package_size - 1]) [[unlikely]]
      return false;

    if constexpr (payload_size > 0) {
      std::copy(response.begin() + 5, response.begin() + 5 + payload_size, payload.begin());
    }

    return true;
  }

  template <typename T>
    requires std::is_trivial_v<T>
  [[nodiscard]] auto read_value(ServoRegister const& reg, T& value) noexcept -> bool {
    static constexpr std::size_t payload_size = sizeof(T);

    std::array const params{
        as_byte(reg),
        static_cast<std::uint8_t>(payload_size),
    };
    send_packet(ServoInstruction::InstRead, params);

    if (std::array<std::uint8_t, payload_size> payload{}; receive_packet(payload)) {
      std::memcpy(&value, payload.data(), payload_size);
      return true;
    }

    return false;
  }

 public:
  constexpr explicit Servo(Driver& driver_uart, PowerEnable& driver_power) noexcept : m_driver_uart(driver_uart), m_driver_power(driver_power) {}

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_driver_uart.init()) [[unlikely]]
      return type::SystemError::ServoInitError;

    if (!m_driver_power.init()) [[unlikely]]
      return type::SystemError::ServoInitError;

    if (!m_driver_power.enable()) [[unlikely]]
      return type::SystemError::ServoPowerFail;

    return type::SystemError::None;
  }

  auto set_position(type::Position const target_position) noexcept -> void {
    static constexpr std::size_t params_size{7};
    static constexpr type::Position position_min{type::Position::value_min};
    static constexpr type::Position position_max{type::Position::value_max};

    type::ServoPosition const servo_position =
        common::map_range(target_position, position_min, position_max, m_calibration_data.position_minimal, m_calibration_data.position_maximal);

    std::array<std::uint8_t, params_size> params{};
    params[0] = as_byte(ServoRegister::TargetPosition);
    params[1] = as_byte(servo_position.value);
    params[2] = as_byte(servo_position.value >> 8);
    params[3] = as_byte(0);
    params[4] = as_byte(0 >> 8);
    params[5] = as_byte(0);
    params[6] = as_byte(0 >> 8);
    send_packet(ServoInstruction::InstWrite, params);

    std::array<std::uint8_t, 0> payload;
    std::ignore = receive_packet(payload);
  }

  [[nodiscard]] auto get_telemetry(type::ServoTelemetry& telemetry) noexcept -> bool {
    static constexpr std::size_t payload_size{30};
    static constexpr std::array<std::uint8_t, 2> params{
        as_byte(ServoRegister::TorqueEnable),
        payload_size,
    };

    send_packet(ServoInstruction::InstRead, params);

    if (std::array<std::uint8_t, payload_size> payload{}; receive_packet(payload)) {
      telemetry.is_connected = true;
      telemetry.is_enabled = payload[0];
      telemetry.is_enabled = payload[0];
      telemetry.position = ((payload[1] << 8) | payload[0]) & 0x7FFF;
      telemetry.speed = ((payload[3] << 8) | payload[2]) & 0x7FFF;
      telemetry.voltage = payload[6];
      telemetry.temperature = payload[7];
      telemetry.is_moved = payload[10];
      telemetry.current = ((payload[14] << 8) | payload[13]) & 0x7FFF;

      return true;
    }

    return false;
  }

  auto set_mode(ServoMode const mode) noexcept -> void {
    std::array<std::uint8_t, 2> params{};
    params[0] = as_byte(ServoRegister::OperationMode);
    params[1] = as_byte(mode);
    send_packet(ServoInstruction::InstWrite, params);

    std::array<std::uint8_t, 0> payload;
    std::ignore = receive_packet(payload);
  }

  [[nodiscard]] auto read_current(type::Current& current) noexcept -> bool { return read_value(ServoRegister::CurrentCurrent, current); }

  [[nodiscard]] auto read_position(type::ServoPosition& position) noexcept -> bool { return read_value(ServoRegister::CurrentPosition, position); }
};

}  // namespace device
