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

#include "config/concepts.hpp"

namespace device {

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

template <class Driver, std::uint16_t Debounce = 1, std::uint16_t LongPress = 20>
  requires concepts::GPIO<Driver>
class Button {
  Driver& m_driver;

  std::uint16_t m_ticks_count{0};
  ButtonState m_state{ButtonState::Idle};
  ButtonEvent m_current_event{ButtonEvent::None};

 public:
  constexpr explicit Button(Driver& driver) noexcept : m_driver{driver} {}

  constexpr Button() noexcept = delete;

  Button(Button const&) noexcept = delete;
  auto operator=(Button const&) noexcept -> Button& = delete;

  Button(Button&&) noexcept = delete;
  auto operator=(Button&&) noexcept -> Button& = delete;

  constexpr ~Button() noexcept = default;

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_driver.init()) [[unlikely]] {
      return type::SystemError::ButtonInitFault;
    }

    return type::SystemError::None;
  }

  auto update() noexcept -> void {
    bool const is_pressed = is_active();

    switch (m_state) {
      case ButtonState::Idle: {
        if (!is_pressed) {
          return;
        }
        m_ticks_count = 0;
        m_state = ButtonState::Debounce;
      } break;

      case ButtonState::Debounce: {
        if (!is_pressed) {
          m_state = ButtonState::Idle;
          return;
        }
        m_ticks_count++;
        if (m_ticks_count >= Debounce) {
          m_state = ButtonState::Pressed;
        }
      } break;

      case ButtonState::Pressed: {
        if (!is_pressed) {
          m_state = ButtonState::Idle;
          m_current_event = ButtonEvent::ShortPress;
          return;
        }
        m_ticks_count++;
        if (m_ticks_count >= LongPress) {
          m_state = ButtonState::WaitRelease;
          m_current_event = ButtonEvent::LongPress;
        }
      } break;

      case ButtonState::WaitRelease: {
        if (!is_pressed) {
          m_state = ButtonState::Idle;
        }
      } break;
    }
  }

  [[nodiscard]] auto is_active() noexcept -> bool { return m_driver.get_level(); }

  [[nodiscard]] auto is_short_press() noexcept -> bool {
    if (m_current_event == ButtonEvent::ShortPress) {
      m_current_event = ButtonEvent::None;
      return true;
    }

    return false;
  }

  [[nodiscard]] auto is_long_press() noexcept -> bool {
    if (m_current_event == ButtonEvent::LongPress) {
      m_current_event = ButtonEvent::None;
      return true;
    }

    return false;
  }
};

}  // namespace device
