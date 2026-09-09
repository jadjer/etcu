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
#include "type/error.hpp"

namespace device {

constexpr auto MakePattern(std::uint8_t const clicks, std::uint8_t const mask) noexcept -> std::uint16_t {
  return (static_cast<std::uint16_t>(clicks) << 8) | mask;
}

enum class ClickType : std::uint8_t { Short = 0, Long = 1 };

template <typename... Args>
constexpr auto BuildPattern(Args... args) noexcept -> std::uint16_t {
  std::uint8_t const count = sizeof...(args);
  std::uint8_t mask = 0;
  ((mask = (mask << 1) | static_cast<std::uint8_t>(args)), ...);
  return MakePattern(count, mask);
}

enum class ButtonState : std::uint8_t { Idle = 0, Pressed, WaitNextPress, WaitReleaseLong };

template <class Driver, std::uint16_t Debounce = 3, std::uint16_t LongPress = 50, std::uint16_t MultiClickTimeout = 35>
  requires concepts::GPIO<Driver>
class Button {
  Driver& m_driver;

  bool m_has_event{false};

  std::uint16_t m_ticks{0};
  std::uint16_t m_hold_ticks{0};
  std::uint8_t m_clicks_count{0};
  std::uint8_t m_pattern_mask{0};
  std::uint16_t m_ready_pattern{0};

  ButtonState m_state{ButtonState::Idle};

  auto change_state(ButtonState const new_state) noexcept -> void {
    m_ticks = 0;
    m_hold_ticks = 0;
    m_state = new_state;
  }

  auto push_click(ClickType type) noexcept -> void {
    m_pattern_mask = (m_pattern_mask << 1) | static_cast<std::uint8_t>(type);
    m_clicks_count++;
  }

  auto reset() noexcept -> void {
    m_clicks_count = 0;
    m_pattern_mask = 0;
    change_state(ButtonState::Idle);
  }

 public:
  constexpr explicit Button(Driver& driver) noexcept : m_driver{driver} {}
  constexpr Button() noexcept = delete;

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_driver.init()) [[unlikely]] {
      return type::SystemError::PeripheralInitError;
    }

    return type::SystemError::None;
  }

  auto update() noexcept -> void {
    bool const is_pressed = m_driver.get_level();

    switch (m_state) {
      case ButtonState::Idle:
        if (is_pressed && ++m_ticks >= Debounce) {
          change_state(ButtonState::Pressed);
        } else if (!is_pressed) {
          m_ticks = 0;
        }
        break;

      case ButtonState::Pressed:
        if (!is_pressed) {
          m_hold_ticks = 0;
          if (++m_ticks >= Debounce) {
            push_click(ClickType::Short);
            change_state(ButtonState::WaitNextPress);
          }
        } else {
          m_ticks = 0;
          if (++m_hold_ticks >= LongPress) {
            push_click(ClickType::Long);
            change_state(ButtonState::WaitReleaseLong);
          }
        }
        break;

      case ButtonState::WaitNextPress:
        if (is_pressed) {
          if (++m_ticks >= Debounce) {
            change_state(ButtonState::Pressed);
          }
        } else {
          m_ticks = 0;
          if (++m_hold_ticks >= MultiClickTimeout) {
            m_ready_pattern = MakePattern(m_clicks_count, m_pattern_mask);
            m_has_event = true;
            reset();
          }
        }
        break;

      case ButtonState::WaitReleaseLong:
        if (!is_pressed && ++m_ticks >= Debounce) {
          change_state(ButtonState::WaitNextPress);
        } else if (is_pressed) {
          m_ticks = 0;
        }
        break;
    }
  }

  [[nodiscard]] auto has_event() noexcept -> bool {
    if (m_has_event) {
      m_has_event = false;
      return true;
    }
    return false;
  }

  [[nodiscard]] auto get_pattern() const noexcept -> std::uint16_t { return m_ready_pattern; }
};

}  // namespace device
