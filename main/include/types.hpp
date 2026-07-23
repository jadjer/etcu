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
  Normal = 0,
  Off,
};

enum class SystemError : std::uint32_t {
  None = 0,

  ServoInitFault = 1 << 0,
  ServoCommsFault = 1 << 2,
  ServoOvercurrent = 1 << 3,
  ServoOvertemp = 1 << 4,

  AcceleratorInitFault = 1 << 10,
  AcceleratorMismatch = 1 << 11,

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
  std::uint16_t rpm{0};
  std::uint16_t tps{0};
  std::uint16_t speed{0};
};

struct ServoTelemetry {
  std::uint16_t position{0};
  std::uint16_t current{0};
  std::uint16_t temperature{0};
};
