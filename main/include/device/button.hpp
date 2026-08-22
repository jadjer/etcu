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

#include "concepts.hpp"
#include "type.hpp"

namespace device {

enum class ButtonEvent : type::Byte {
  None = 0,
  ShortPress,
  LongPress,
};

enum class ButtonState : type::Byte {
  Idle = 0,
  Debounce,
  Pressed,
  WaitRelease,
};

template <class Driver, type::Ticks debounce = 1, type::Ticks longPress = 20>
  requires concepts::GPIO<Driver> && concepts::Ticks<debounce, 1, 10> && concepts::Ticks<longPress, 10, 100>

class Button {
  Driver& m_driver;

  type::Ticks m_ticks_count{0};
  ButtonState m_state{ButtonState::Idle};
  ButtonEvent m_current_event{ButtonEvent::None};

 public:
  constexpr explicit Button(Driver& driver) noexcept : m_driver{driver} {}

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_driver.init()) [[unlikely]]
      return type::SystemError::ButtonInitFault;

    return type::SystemError::None;
  }

  auto update() noexcept -> void {
    bool const is_pressed = is_active();

    switch (m_state) {
      case ButtonState::Idle: {
        if (!is_pressed) {
          m_current_event = ButtonEvent::None;
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

        if (m_ticks_count >= debounce) {
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

        if (m_ticks_count >= longPress) {
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

  [[nodiscard]] auto is_short_press() const noexcept -> bool { return m_current_event == ButtonEvent::ShortPress; }

  [[nodiscard]] auto is_long_press() const noexcept -> bool { return m_current_event == ButtonEvent::LongPress; }
};

}  // namespace device
