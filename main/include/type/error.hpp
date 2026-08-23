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
// Created by jadjer on 24.08.26.
//

#pragma once

#include "type/primitive.hpp"

namespace type {

enum class ServoError : primitive::Byte {
  None = 0x00,
  Voltage = 0x01,
  AngleLimit = 0x02,
  Overheat = 0x04,
  Overload = 0x08,
  Encoder = 0x10,
  Driver = 0x20,
};

[[nodiscard]] constexpr auto operator&(ServoError const lhs, ServoError const rhs) noexcept -> bool {
  return (static_cast<primitive::Byte>(lhs) & static_cast<primitive::Byte>(rhs)) != 0;
}

enum class SystemError : primitive::Error {
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
  ServoPowerFail = 1 << 15,

  AcceleratorInitFault = 1 << 16,
  AcceleratorCalibrateFault = 1 << 17,
  AcceleratorReadFault = 1 << 18,
  AcceleratorMismatch = 1 << 19,
  ButtonInitFault = 1 << 20,
  ButtonReadFault = 1 << 21,
  ECUInitFault = 1 << 22,
  IndicatorInitFault = 1 << 23,
  BluetoothInitFault = 1 << 24,
  BluetoothSetPowerFault = 1 << 25,
  BluetoothSetMTUFault = 1 << 26,
  BluetoothConnectedFault = 1 << 27,
};

[[nodiscard]] constexpr auto operator|(SystemError const a, SystemError const b) -> SystemError {
  return static_cast<SystemError>(static_cast<primitive::Error>(a) | static_cast<primitive::Error>(b));
}

[[nodiscard]] constexpr auto has_error(SystemError const err) -> bool {
  return static_cast<primitive::Error>(err) != 0;
}

[[nodiscard]] constexpr auto has_error(SystemError const mask, SystemError const err) -> bool {
  return (static_cast<primitive::Error>(mask) & static_cast<primitive::Error>(err)) != 0;
}

}

