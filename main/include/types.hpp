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

enum class Mode : std::uint8_t {
  Off = 0,
  Normal,
  Emergency,
  Calibration,
};

enum class SystemError : Error {
  None = 0,
  ServoInitFault = 1 << 0,
  ServoCommsFault = 1 << 1,
  ServoOvercurrent = 1 << 2,
  ServoOvertemp = 1 << 3,
  ServoMechanicalFault = 1 << 4,
  AcceleratorInitFault = 1 << 5,
  AcceleratorCalibrateFault = 1 << 6,
  AcceleratorReadFault = 1 << 7,
  AcceleratorMismatch = 1 << 8,
  ButtonInitFault = 1 << 9,
  ButtonReadFault = 1 << 10,
  ECUInitFault = 1 << 11,
  IndicatorInitFault = 1 << 12,
  GuardLock = 1 << 30,
};

constexpr auto operator|(SystemError const a, SystemError const b) -> SystemError {
  return static_cast<SystemError>(static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b));
}

constexpr auto has_error(SystemError const mask, SystemError const err) -> bool {
  return (static_cast<std::uint32_t>(mask) & static_cast<std::uint32_t>(err)) != 0;
}

struct SharedData {
  bool clutch_pressed{false};
  bool brake_pressed{false};
  bool guard_active{false};

  RPM rpm{0};
  Position tps{0};
  Speed speed{0};
};

struct ServoTelemetry {
  Current current{0};
  Position position{0};
  Temperature temperature{0};
};

struct CalibrationData {
  std::uint32_t struct_version = 0x10000001;
  MilliVolt hall_a_minimal;
  MilliVolt hall_a_maximal;
  MilliVolt hall_b_minimal;
  MilliVolt hall_b_maximal;
};
