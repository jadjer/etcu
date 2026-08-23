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
#include "type/type.hpp"
#include "common/map_range.hpp"

namespace device {

enum class ServoRegister : type::primitive::Byte {
  // =========================================================================
  // ЗОНА EEPROM (Энергонезависимая. Запись доступна только при RegLockSign = 0)
  // =========================================================================
  RegModel = 0x03,             // 1 байт (R) - Модель (у ST3020 свой ID модели)
  RegId = 0x05,                // 1 байт - Идентификатор сервопривода (ID: 1-253)
  RegBaudRate = 0x06,          // 1 байт - Скорость UART (0: 1Mbps, 1: 500kbps, 2: 250kbps, 3: 128kbps, 4: 115.2kbps, 5: 76.8kbps, 6: 57.6kbps, 7: 38.4kbps)
  RegMinPositionLimit = 0x09,  // 2 байта - Минимальный предел угла (0 - 4095)
  RegMaxPositionLimit = 0x0B,  // 2 байта - Максимальный предел угла (0 - 4095)
  RegCwDeadband = 0x1A,        // 1 байт - Люфт/мертвая зона по часовой стрелке
  RegCcwDeadband = 0x1B,       // 1 байт - Люфт/мертвая зона против часовой стрелки
  RegOffset = 0x1F,            // 2 байта - Коррекция программного нуля (смещение средней точки)
  RegMode = 0x21,              // 1 байт - Режим работы устройства (0: Position Mode, 1: Speed/Wheel Mode, 2: PWM Mode, 3: Step Mode)

  // =========================================================================
  // ЗОНА RAM (Оперативная. Сбрасывается в дефолт при выключении питания)
  // =========================================================================
  RegTorqueEnable = 0x28,    // 1 байт - Включение H-моста мотора (1: Включен, 0: Выключен/Вал свободен)
  RegAcceleration = 0x29,    // 1 байт - Ускорение старта и стопа (Smooth профиль движения, 0 - выкл)
  RegTargetPosition = 0x2A,  // 2 байта - Целевое положение (0 - 4095 шагов)
  RegTargetTime = 0x2C,      // 2 байта - Время, за которое сервопривод обязан доехать до цели (в мс)
  RegTargetSpeed = 0x2E,     // 2 байта - Целевая скорость движения (шагов в секунду)
  RegTorqueLimit = 0x30,     // 2 байта - Предел крутящего момента (0 - 1023, где 1023 - 100% мощности)
  RegLock = 0x37,            // 1 байт - Блокировка EEPROM (1: Заблокировано [Дефолт], 0: Разрешена запись)

  // Регистры обратной связи/телеметрии (Только чтение)
  RegPresentPosition = 0x38,  // 2 байта (R) - Текущее положение магнитного энкодера (0 - 4095)
  RegPresentSpeed = 0x3A,     // 2 байта (R) - Текущая скорость вала (шагов/сек). Специфический знак направления.
  RegPresentLoad = 0x3C,      // 2 байта (R) - Текущая расчетная нагрузка (0 - 1023, 10-й бит задает вектор)
  RegPresentVoltage = 0x3E,   // 1 байт (R) - Текущее напряжение питания (значение / 10 = Вольты, например 120 = 12.0V)
  RegPresentTemp = 0x3F,      // 1 байт (R) - Текущая температура внутри корпуса (в °C)
  RegMoveStatus = 0x42,       // 1 байт (R) - Флаг движения (1: вал еще крутится, 0: приехали в целевую точку)
  RegPresentCurrent = 0x45,   // 2 байта (R) - Честный физический ток на шунтах (в мА, 15-й бит — направление)

  // Дополнительные глубокие диагностические регистры ST3020
  RegEncoderRawPosition = 0x44,  // 2 байта (R) - Физические сырые данные энкодера без учета калибровочного Offset
  RegPresentPwm = 0x46,          // 2 байта (R) - Текущая заполненность ШИМ, подаваемая драйвером на фазы мотора
  RegStatusErrorFlags = 0x48,    // 1 байт (R) - Битовая маска активных ошибок аппаратной защиты
};

enum class ServoInstruction : type::primitive::Byte {
  InstPing = 0x01,
  InstRead = 0x02,
  InstWrite = 0x03,
  InstRegWrite = 0x04,
  InstAction = 0x05,
  InstReset = 0x06,
  InstSyncWrite = 0x83
};

template <typename T>
  requires std::is_enum_v<T>
[[nodiscard]] constexpr auto operator+(T const reg) noexcept -> type::primitive::Byte {
  return static_cast<type::primitive::Byte>(reg);
}

template <class Driver, class PowerEnable, type::primitive::ServoId const servoId = 1>
  requires concepts::UART<Driver> && concepts::GPIO<PowerEnable>

class Servo {
  type::ServoCalibrationData m_calibration_data{
      .position_minimal{600},
      .position_maximal{1250},
  };

  Driver& m_driver_uart;
  PowerEnable& m_driver_power;

  template <type::primitive::Size packetSize>
    requires(packetSize >= 6)
  [[nodiscard]] auto calculate_checksum_for_packet(std::array<type::primitive::Byte, packetSize> const bytes) const noexcept -> type::primitive::Byte {
    static constexpr type::primitive::Size START_INDEX = 2;
    static constexpr type::primitive::Size END_INDEX = packetSize - 1;

    std::uint32_t const sum = std::accumulate(bytes.begin() + START_INDEX, bytes.begin() + END_INDEX, 0U);
    return static_cast<type::primitive::Byte>(~(sum & 0xFF));
  }

  template <type::primitive::Size paramSize>
  auto send_packet(ServoInstruction const instruction, std::array<type::primitive::Byte, paramSize> const& parameters) const noexcept -> void {
    type::primitive::Size constexpr packet_size = paramSize + 6;
    std::array<type::primitive::Byte, packet_size> packet;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servoId;
    packet[3] = paramSize + 2;
    packet[4] = +instruction;

    if constexpr (paramSize > 0) {
      [[likely]]
      for (type::primitive::Size i = 0; i < paramSize; ++i) {
        packet[5 + i] = parameters[i];
      }
    }

    packet[5 + paramSize] = calculate_checksum_for_packet(packet);

    std::ignore = m_driver_uart.flush();
    std::ignore = m_driver_uart.template write<packet_size>(packet);
  }

  template <type::primitive::Size paramSize>
  [[nodiscard]] auto receive_packet(std::array<type::primitive::Byte, paramSize>& payload) noexcept -> bool {
    static constexpr type::primitive::Size PACKET_SIZE = paramSize + 6;

    if (servoId == 0xFE)
      return false;

    std::array<type::primitive::Byte, PACKET_SIZE> response;

    if (int const read_bytes = m_driver_uart.template read<PACKET_SIZE>(response, 30); std::cmp_less(read_bytes, PACKET_SIZE)) {
      [[unlikely]] return false;
    }

    if (response[0] != 0xFF || response[1] != 0xFF || response[2] != servoId) {
      [[unlikely]] return false;
    }

    type::primitive::Byte const calculated_check_sum = calculate_checksum_for_packet(response);

    if (calculated_check_sum != response[PACKET_SIZE - 1]) {
      [[unlikely]] return false;
    }

    if constexpr (paramSize > 0) {
      [[likely]]
      for (std::size_t i = 0; i < paramSize; ++i) {
        payload[i] = static_cast<type::primitive::Byte>(response[5 + i]);
      }
    }

    return true;
  }

 public:
  constexpr explicit Servo(Driver& driver_uart, PowerEnable& driver_power) noexcept : m_driver_uart(driver_uart), m_driver_power(driver_power) {}

  [[nodiscard]] constexpr auto init() noexcept -> type::SystemError {
    if (!m_driver_uart.init()) [[unlikely]]
      return type::SystemError::ServoInitError;

    if (!m_driver_power.init()) [[unlikely]]
      return type::SystemError::ServoInitError;

    if (!m_driver_power.enable()) [[unlikely]]
      return type::SystemError::ServoPowerFail;

    return type::SystemError::None;
  }

  constexpr auto set_position(type::Position const target_position) noexcept -> void {
    type::ServoPosition const servo_position =
        common::map_range(target_position, type::Position{type::Position::MinValue}, type::Position{type::Position::MaxValue},
                          m_calibration_data.position_minimal, m_calibration_data.position_maximal);

    std::array<type::primitive::Byte, 7> params{};
    params[0] = +ServoRegister::RegTargetPosition;
    params[1] = servo_position.value & 0xFF;
    params[2] = (servo_position.value >> 8) & 0xFF;
    params[3] = 0 & 0xFF;
    params[4] = (0 >> 8) & 0xFF;
    params[5] = 0 & 0xFF;
    params[6] = (0 >> 8) & 0xFF;
    send_packet(ServoInstruction::InstWrite, params);

    std::array<type::primitive::Byte, 0> payload;
    std::ignore = receive_packet(payload);
  }

  [[nodiscard]] constexpr auto get_telemetry(type::ServoTelemetry& telemetry) noexcept -> bool {
    std::array<type::primitive::Byte, 2> constexpr params{+ServoRegister::RegPresentPosition, 15};
    send_packet(ServoInstruction::InstRead, params);

    if (std::array<type::primitive::Byte, 15> payload{}; receive_packet(payload)) {
      telemetry.is_connected = true;
      telemetry.position = ((payload[1] << 8) | payload[0]) & 0x7FFF;
      telemetry.speed = ((payload[3] << 8) | payload[2]) & 0x7FFF;
      telemetry.load = ((payload[5] << 8) | payload[4]) & 0x03FF;
      telemetry.voltage = payload[6];
      telemetry.temperature = payload[7];
      telemetry.is_moved = payload[10];
      telemetry.current = ((payload[14] << 8) | payload[13]) & 0x7FFF;

      return true;
    }

    return false;
  }

  constexpr auto set_mode(type::ServoMode const mode) noexcept -> void {
    std::array const params{+ServoRegister::RegMode, +mode};
    send_packet(ServoInstruction::InstWrite, params);

    std::array<type::primitive::Byte, 0> payload;
    std::ignore = receive_packet(payload);
  }

  constexpr auto set_speed(std::int16_t const speed) noexcept -> void {
    auto const abs_speed = static_cast<int16_t>(std::abs(speed));

    std::uint16_t reg_value = abs_speed;

    if (speed < 0) {
      reg_value |= 0x8000;
    }

    auto const low_byte = static_cast<type::primitive::Byte>(reg_value & 0xFF);
    auto const high_byte = static_cast<type::primitive::Byte>((reg_value >> 8) & 0xFF);

    std::array const params{+ServoRegister::RegTargetSpeed, low_byte, high_byte};
    send_packet(ServoInstruction::InstWrite, params);

    std::array<type::primitive::Byte, 0> payload;
    std::ignore = receive_packet(payload);
  }

  [[nodiscard]] constexpr auto read_current(type::Current& current) noexcept -> bool {
    std::array<type::primitive::Byte, 2> constexpr params{+ServoRegister::RegPresentCurrent, 2};
    send_packet(ServoInstruction::InstRead, params);

    if (std::array<type::primitive::Byte, 2> payload{}; receive_packet(payload)) {
      current = ((payload[1] << 8) | payload[0]) & 0x7FFF;

      return true;
    }

    return false;
  }

  [[nodiscard]] constexpr auto read_position(type::ServoPosition& position) noexcept -> bool {
    std::array<type::primitive::Byte, 2> constexpr params{+ServoRegister::RegPresentPosition, 2};
    send_packet(ServoInstruction::InstRead, params);

    if (std::array<type::primitive::Byte, 2> payload{}; receive_packet(payload)) {
      position = payload[0] | (payload[1] << 8);

      return true;
    }

    return false;
  }
};

}  // namespace device
