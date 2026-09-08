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

enum class IndicatorMode { Off, On, Blink };

template <class Driver>
  requires concepts::GPIO<Driver>
class Indicator {
  static constexpr std::uint8_t ticks_per_half_period = 25;

  Driver& m_driver;

  IndicatorMode m_mode{IndicatorMode::Off};
  IndicatorMode m_target_mode{IndicatorMode::Off};

  bool m_current_state{false};
  std::size_t m_tick_counter{0};
  std::size_t m_blink_counter{0};

  auto apply_state(bool const state) noexcept -> bool {
    m_current_state = state;
    return m_driver.set_level(state);
  }

 public:
  constexpr explicit Indicator(Driver& driver) : m_driver(driver) {}

  constexpr Indicator() noexcept = delete;

  Indicator(Indicator const&) noexcept = delete;
  auto operator=(Indicator const&) noexcept -> Indicator& = delete;

  Indicator(Indicator&&) noexcept = delete;
  auto operator=(Indicator&&) noexcept -> Indicator& = delete;

  constexpr ~Indicator() noexcept = default;

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_driver.init()) [[unlikely]] {
      return type::SystemError::IndicatorInitFault;
    }

    apply_state(false);

    return type::SystemError::None;
  }

  auto update() noexcept -> bool {
    if (m_mode != IndicatorMode::Blink) {
      return false;
    }

    m_tick_counter++;

    if (m_tick_counter < ticks_per_half_period) {
      return true;
    }

    m_tick_counter = 0;

    if (m_blink_counter > 0) {
      apply_state(!m_current_state);
      m_blink_counter--;
    } else {
      m_mode = m_target_mode;
      apply_state(m_mode == IndicatorMode::On);
    }

    return m_mode == IndicatorMode::Blink;
  }

  auto turn_on() noexcept -> bool {
    m_target_mode = IndicatorMode::On;

    if (m_mode != IndicatorMode::Blink) {
      m_mode = IndicatorMode::On;
      return apply_state(true);
    }

    return true;
  }

  auto turn_off() noexcept -> bool {
    m_target_mode = IndicatorMode::Off;

    if (m_mode != IndicatorMode::Blink) {
      m_mode = IndicatorMode::Off;
      return apply_state(false);
    }

    return true;
  }

  auto blink_times(std::size_t const count) noexcept -> bool {
    if (count == 0) {
      m_mode = m_target_mode;
      return apply_state(m_mode == IndicatorMode::On);
    }

    m_mode = IndicatorMode::Blink;
    m_blink_counter = count * 2 - 1;
    m_tick_counter = 0;

    return apply_state(true);
  }
};

}  // namespace device
