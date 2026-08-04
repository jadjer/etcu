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

#include "constants.hpp"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"

using CoreRate = std::uint8_t;
using CoreId = std::uint8_t;
using Ticks = std::uint16_t;
using MilliSec = std::uint16_t;
using Voltage = std::uint16_t;
using MilliVolt = std::uint16_t;
using GPIONum = gpio_num_t;
using UARTPort = uart_port_t;
using ADCUnit = adc_unit_t;
using ADCChannel = adc_channel_t;
using Position = std::uint8_t;
using ServoPosition = std::uint16_t;
using RPM = std::uint16_t;
using Speed = std::uint16_t;
using Current = std::uint16_t;
using Temperature = std::uint16_t;
using Error = std::uint32_t;
using Byte = std::uint8_t;
using Load = std::uint16_t;
using Time = std::uint16_t;
using ServoId = std::uint8_t;

enum class ButtonEvent : Byte {
  None = 0,
  ShortPress,
  LongPress,
};

enum class ButtonState : Byte {
  Idle = 0,
  Debounce,
  Pressed,
  WaitRelease,
};

enum class SystemState : Byte {
  Off = 0,
  Normal,
  Calibration,
  Update,
};

enum class OTAStatus : Byte {
  ReadyForNext = 0x00,
  Busy = 0x01,
  ErrorOccurred = 0x02,
  Completed = 0x03,
};

enum class ServoMode : Byte {
  PositionMode = 0x00,
  WheelMode = 0x01,
  PwmMode = 0x02,
  StepMode = 0x03,
};

enum class ServoError : Byte {
  None = 0x00,
  Voltage = 0x01,
  AngleLimit = 0x02,
  Overheat = 0x04,
  Overload = 0x08,
  Encoder = 0x10,
  Driver = 0x20,
};

constexpr auto operator&(ServoError const lhs, ServoError const rhs) noexcept -> bool {
  return (static_cast<std::uint8_t>(lhs) & static_cast<std::uint8_t>(rhs)) != 0;
}

enum class SystemError : Error {
  None = 0,

  GuardLock = 1 << 0,

  ServoInitError = 1 << 1,
  ServoCommsError = 1 << 2,
  ServoProtocolError = 1 << 3,
  ServoCheckSumError = 1 << 4,
  ServoReadError = 1 << 5,
  ServoWriteError = 1 << 6,
  ServoModeError = 1 << 7,
  ServoSpeedError = 1 << 8,
  ServoPositionError = 1 << 9,
  ServoCurrentError = 1 << 10,
  ServoTorqueError = 1 << 11,
  ServoOvercurrent = 1 << 12,
  ServoOvertemp = 1 << 13,
  ServoCalibrateError = 1 << 14,

  AcceleratorInitFault = 1 << 15,
  AcceleratorCalibrateFault = 1 << 16,
  AcceleratorReadFault = 1 << 17,
  AcceleratorMismatch = 1 << 18,
  ButtonInitFault = 1 << 19,
  ButtonReadFault = 1 << 20,
  ECUInitFault = 1 << 21,
  IndicatorInitFault = 1 << 22,
  BluetoothInitFault = 1 << 23,
  BluetoothSetPowerFault = 1 << 24,
  BluetoothSetMTUFault = 1 << 25,
  BluetoothConnectedFault = 1 << 26,
};

[[nodiscard]] constexpr auto operator|(SystemError const a, SystemError const b) -> SystemError {
  return static_cast<SystemError>(static_cast<Error>(a) | static_cast<Error>(b));
}

[[nodiscard]] constexpr auto has_error(SystemError const err) -> bool {
  return (static_cast<Error>(err)) != 0;
}

[[nodiscard]] constexpr auto has_error(SystemError const mask, SystemError const err) -> bool {
  return (static_cast<Error>(mask) & static_cast<Error>(err)) != 0;
}

enum class ServoRegister : Byte {
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

enum class ServoInstruction : Byte {
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
[[nodiscard]] constexpr auto operator+(T const reg) noexcept -> Byte {
  return static_cast<Byte>(reg);
}

struct BluetoothControl {
  bool sync_enabled{false};
  Position accelerator_offset{0};
};

struct OTAChunk {
  std::array<Byte, MAX_BLE_PAYLOAD_SIZE> chunk{};
  std::uint16_t chunk_size{0};
  std::uint16_t chunk_number{0};
  std::uint16_t chunk_total{0};
  std::uint32_t firmware_size{0};
};

struct AcceleratorCalibrationData {
  std::uint32_t struct_version = 0x10000001;

  MilliVolt hall_a_minimal;
  MilliVolt hall_a_maximal;
  MilliVolt hall_b_minimal;
  MilliVolt hall_b_maximal;

  static auto constexpr StructName = "acc_calib";
};

struct ServoCalibrationData {
  std::uint32_t struct_version = 0x10000001;

  ServoPosition position_minimal;
  ServoPosition position_maximal;

  static auto constexpr StructName = "servo_calib";
};

struct ServoTelemetry {
  bool is_connected{false};
  bool is_moved{false};

  Load load{0};
  Speed speed{0};
  Current current{0};
  Voltage voltage{0};
  ServoPosition position{0};
  Temperature temperature{0};
};

struct ECUTelemetry {
  bool is_connected{false};

  RPM rpm{0};
  Speed speed{0};
  Position tps{0};
  bool started{false};
  bool clutch_enabled{false};
};

struct DriveTelemetry {
  ServoTelemetry servo_telemetry{};
  Position accelerator_position{0};
  Position throttle_position{0};
};

struct SystemTelemetry {
  ServoTelemetry servo_telemetry{};
  ECUTelemetry ecu_telemetry{};
  Position accelerator_position{0};
  Position accelerator_offset{0};
  Position throttle_position{0};
  Speed target_speed{0};
  bool guard_active{false};
  bool brake_enabled{false};
  bool clutch_enabled{false};
  SystemState system_state{SystemState::Off};
  SystemError system_errors{SystemError::None};
};
