//
// Created by jadjer on 23.07.26.
//

#pragma once

#include <cstdint>

enum class ButtonEvent : std::uint8_t { None = 0, ShortPress, LongPress };

enum class ButtonState : std::uint8_t { Idle, Debounce, Pressed, WaitRelease };

enum class Mode : std::uint8_t {
  Normal = 0,
  Eco,
  Emergency,
};

enum class SystemError : std::uint8_t {
  None = 0,
  AcceleratorMismatch = 1 << 0,
  ServoCommsFault = 1 << 1,
  ServoOvercurrent = 1 << 2,
  ServoOvertemperature = 1 << 3,
  EcuCommsFault = 1 << 4
};

constexpr auto operator|(SystemError const a, SystemError const b) -> SystemError {
  return static_cast<SystemError>(static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b));
}

constexpr auto has_error(SystemError const mask, SystemError const err) -> bool {
  return (static_cast<std::uint32_t>(mask) & static_cast<std::uint32_t>(err)) != 0;
}

struct DriverInput {
  uint16_t raw_hall_1{0};
  uint16_t raw_hall_2{0};
  bool clutch_pressed{false};
  bool brake_pressed{false};
  bool guard_active{false};
};
