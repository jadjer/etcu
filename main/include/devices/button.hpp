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

#include <esp_timer.h>
#include "types.hpp"

namespace devices {

template <gpio_num_t Pin, bool ActiveLow = true>
class Button {
  static std::uint32_t constexpr DebounceTicks = 2;
  static std::uint32_t constexpr LongPressTicks = 50;

  ButtonState m_state{ButtonState::Idle};
  uint32_t m_ticks_count{0};
  ButtonEvent m_current_event{ButtonEvent::None};

 public:
  void init() noexcept {
    gpio_config_t const config = {
        .pin_bit_mask = (1ULL << Pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = ActiveLow ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = ActiveLow ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&config);
  }

  void update() noexcept {
    bool const is_pressed = (gpio_get_level(Pin) == (ActiveLow ? 0 : 1));
    m_current_event = ButtonEvent::None;

    switch (m_state) {
      case ButtonState::Idle:
        if (is_pressed) {
          m_state = ButtonState::Debounce;
          m_ticks_count = 0;
        }
        break;

      case ButtonState::Debounce:
        if (is_pressed) {
          m_ticks_count++;
          if (m_ticks_count >= DebounceTicks) {
            m_state = ButtonState::Pressed;
            m_ticks_count = 0;
          }
        } else {
          m_state = ButtonState::Idle;
        }
        break;

      case ButtonState::Pressed:
        if (is_pressed) {
          m_ticks_count++;
          // Генерируем событие LongPress сразу по истечении времени (не дожидаясь отпускания)
          if (m_ticks_count >= LongPressTicks) {
            m_current_event = ButtonEvent::LongPress;
            m_state = ButtonState::WaitRelease;
          }
        } else {
          // Кнопку отпустили раньше LongPress — фиксируем короткое нажатие
          m_current_event = ButtonEvent::ShortPress;
          m_state = ButtonState::Idle;
        }
        break;

      case ButtonState::WaitRelease:
        if (!is_pressed) {
          m_state = ButtonState::Idle;
        }
        break;
    }
  }

  // Неблокирующие методы проверки флагов для конечного автомата контроллера
  [[nodiscard]] auto is_short_press() const noexcept -> bool {
    return m_current_event == ButtonEvent::ShortPress;
  }

  [[nodiscard]] auto is_long_press() const noexcept -> bool {
    return m_current_event == ButtonEvent::LongPress;
  }
};

}  // namespace devices
