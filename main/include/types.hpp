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
using MilliVolt = std::uint16_t;
using GPIONum = gpio_num_t;
using UARTPort = uart_port_t;
using ADCUnit = adc_unit_t;
using ADCChannel = adc_channel_t;
using Position = std::uint8_t;
using RPM = std::uint16_t;
using Speed = std::uint16_t;
using Current = std::uint16_t;
using Temperature = std::uint16_t;
using Error = std::uint32_t;
using Byte = std::uint8_t;

enum class ButtonEvent : std::uint8_t {
  None = 0,
  ShortPress,
  LongPress,
};

enum class ButtonState : std::uint8_t {
  Idle = 0,
  Debounce,
  Pressed,
  WaitRelease,
};

enum class SystemState : std::uint8_t {
  Off = 0,
  Normal,
  Calibration,
  Update,
};

enum class OTAStatus : std::uint8_t {
  ReadyForNext = 0x00,
  Busy = 0x01,
  ErrorOccurred = 0x02,
  Completed = 0x03,
};

enum class SystemError : Error {
  None = 0,
  GuardLock = 1 << 0,
  ServoInitFault = 1 << 1,
  ServoCommsFault = 1 << 2,
  ServoOvercurrent = 1 << 3,
  ServoOvertemp = 1 << 4,
  ServoMechanicalFault = 1 << 5,
  AcceleratorInitFault = 1 << 6,
  AcceleratorCalibrateFault = 1 << 7,
  AcceleratorReadFault = 1 << 8,
  AcceleratorMismatch = 1 << 9,
  ButtonInitFault = 1 << 10,
  ButtonReadFault = 1 << 11,
  ECUInitFault = 1 << 12,
  IndicatorInitFault = 1 << 13,
  BluetoothInitFault = 1 << 14,
  BluetoothSetPowerFault = 1 << 15,
  BluetoothSetMTUFault = 1 << 16,
  BluetoothConnectedFault = 1 << 17,
};

// Карта регистров RAM (Little-Endian)
enum class ServoRegister : std::uint8_t {
  RegTorqueEnable = 0x28,     // 1 байт
  RegTargetPosition = 0x2A,   // 2 байта
  RegTargetTime = 0x2C,       // 2 байта
  RegTargetSpeed = 0x2E,      // 2 байта
  RegLockSign = 0x30,         // 1 байт
  RegPresentPosition = 0x38,  // 2 байта (R)
  RegPresentSpeed = 0x3A,     // 2 байта (R)
  RegPresentLoad = 0x3C,      // 2 байта (R)
  RegPresentVoltage = 0x3E,   // 1 байт (R)
  RegPresentTemp = 0x3F,      // 1 байт (R)
  RegMoveStatus = 0x41,       // 1 байт (R)
  RegPresentCurrent = 0x42    // 2 байта (R)
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

constexpr auto operator|(SystemError const a, SystemError const b) -> SystemError {
  return static_cast<SystemError>(static_cast<Error>(a) | static_cast<Error>(b));
}

constexpr auto has_error(SystemError const mask, SystemError const err) -> bool {
  return (static_cast<std::uint32_t>(mask) & static_cast<std::uint32_t>(err)) != 0;
}

struct BluetoothControl {
  bool error_reset{false};
  bool sync_enabled{false};
};

struct OTAChunk {
  std::array<Byte, MAX_BLE_PAYLOAD_SIZE> chunk{0};
  std::uint16_t chunk_size{0};
  std::uint16_t chunk_number{0};
  std::uint16_t chunk_total{0};
  std::uint32_t firmware_size{0};
};

struct CalibrationData {
  std::uint32_t struct_version = 0x10000001;
  MilliVolt twist_a_minimal;
  MilliVolt twist_a_maximal;
  MilliVolt twist_b_minimal;
  MilliVolt twist_b_maximal;
};

struct ServoTelemetry {
  bool connected{false};
  Current current{0};
  Position position{0};
  Temperature temperature{0};
};

struct ECUTelemetry {
  bool connected{false};
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
