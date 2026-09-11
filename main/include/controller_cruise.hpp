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
// Created by jadjer on 11.09.26.
//

#pragma once

#include <algorithm>
#include <cmath>
#include "common/atomic_container.hpp"
#include "common/map_range.hpp"
#include "common/pid_regulator.hpp"
#include "type/control.hpp"
#include "type/type.hpp"

class ControllerCruise {
  static constexpr float dt_ecu{0.1f};
  static constexpr float position_min_f{static_cast<float>(type::Position::value_min)};
  static constexpr float position_max_f{static_cast<float>(type::Position::value_max)};

  common::AtomicContainer<bool> m_is_active{false};
  common::AtomicContainer<float> m_pid_output{0.0f};
  common::AtomicContainer<type::Speed> m_target_speed{type::Speed{0}};

  bool m_was_active_critical{false};
  type::Position m_last_servo{type::Position::value_min};

  common::PidController m_pid{10.0f, 0.4f, 2.0f, dt_ecu, -position_max_f, position_max_f};

 public:
  constexpr ControllerCruise() noexcept = default;

  auto calculate_pid_target(type::Speed const current_speed, type::Control const& control) noexcept -> void {
    type::Speed const target_speed = m_target_speed.load();

    m_pid.set_coefficients(control.cruise.p, control.cruise.i, control.cruise.d);

    float const error = target_speed.as<float>() - current_speed.as<float>();
    float const pid_output = m_pid.update(error, false);

    m_pid_output.store(pid_output);
  }

  [[nodiscard]] auto generate_servo_position(type::Position const accelerator,
                                             bool const safety,
                                             type::Control const& control,
                                             type::Speed const current_speed) noexcept -> type::Position {
    bool is_active = m_is_active.load();

    if (is_active && safety) [[unlikely]] {
      is_active = false;
      m_is_active.store(false);
    }

    type::Position const max_change = control.cruise.limiter;

    type::Position const driver_pos = common::map_range(
        accelerator, control.accelerator.min, control.accelerator.max, control.servo.min, control.servo.max);

    // Безопасная инициализация m_last_error при фронте активации круиза (обнуление Kd)
    if (is_active && !m_was_active_critical) {
      float const initial_error = m_target_speed.load().as<float>() - current_speed.as<float>();
      m_pid.reset_to(driver_pos.as<float>(), initial_error);
      m_last_servo = driver_pos;
      m_was_active_critical = true;
    }

    float const clamped_pid = std::clamp(m_pid_output.load(), position_min_f, position_max_f);
    type::Position const cruise_pos{static_cast<type::primitive::Position>(std::roundf(clamped_pid))};

    // Логика перехвата: если рука водителя открыта сильнее расчетной позиции круиза
    if (is_active && (driver_pos < cruise_pos)) {
      auto const last_servo_raw = m_last_servo.as<std::int32_t>();
      auto const max_change_raw = max_change.as<std::int32_t>();

      m_last_servo = type::Position{std::clamp(cruise_pos.as<std::int32_t>(),
                                               last_servo_raw - max_change_raw,
                                               last_servo_raw + max_change_raw)};
    } else {
      m_last_servo = driver_pos;
      if (!is_active) {
        m_was_active_critical = false;
        m_pid.reset_to(driver_pos.as<float>(), 0.0f); // Теневое копирование ручки
      }
    }

    return m_last_servo;
  }

  auto set_active(bool const active) noexcept -> void { m_is_active.store(active); }
  auto set_target_speed(type::Speed const speed) noexcept -> void { m_target_speed.store(speed); }

  [[nodiscard]] auto is_active() const noexcept -> bool { return m_is_active.load(); }
  [[nodiscard]] auto get_target_speed() const noexcept -> type::Speed { return m_target_speed.load(); }
};
