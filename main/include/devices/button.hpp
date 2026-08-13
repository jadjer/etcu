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
#include "types.hpp"

namespace devices {

template <gpio_num_t Pin, bool Inverse = false, Ticks Debounce = 1, Ticks LongPress = 20>
  requires concepts::TicksConcept<Debounce, 1, 10> && concepts::TicksConcept<LongPress, 10, 100>
class Button {
  Ticks m_ticks_count{0};
  ButtonState m_state{ButtonState::Idle};
  ButtonEvent m_current_event{ButtonEvent::None};

 public:
  auto init() noexcept -> SystemError {
    gpio_config_t const config = {
        .pin_bit_mask = 1ULL << Pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = Inverse ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = Inverse ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (auto const err = gpio_config(&config); err != ESP_OK) {
      return SystemError::ButtonInitFault;
    }

    return SystemError::None;
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

  [[nodiscard]] auto is_active() noexcept -> bool {
    int const level = gpio_get_level(Pin);
    return Inverse ? level == 0 : level == 1;
  }

  [[nodiscard]] auto is_short_press() const noexcept -> bool { return m_current_event == ButtonEvent::ShortPress; }

  [[nodiscard]] auto is_long_press() const noexcept -> bool { return m_current_event == ButtonEvent::LongPress; }
};

}  // namespace devices
