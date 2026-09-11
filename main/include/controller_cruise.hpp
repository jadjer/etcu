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
#include "common/range.hpp"
#include "type/control.hpp"
#include "type/type.hpp"

class PidController {
  float const m_dt, m_min, m_max;

  common::AtomicContainer<float> m_kp{0.0f};
  common::AtomicContainer<float> m_ki{0.0f};
  common::AtomicContainer<float> m_kd{0.0f};
  common::AtomicContainer<float> m_integral{0.0f};
  common::AtomicContainer<float> m_last_error{0.0f};

 public:
  constexpr explicit PidController(float const dt, float const min, float const max) noexcept : m_dt{dt}, m_min{min}, m_max{max} {}

  auto set_coefficients(float const kp, float const ki, float const kd) noexcept -> void {
    m_kp.store(kp);
    m_ki.store(ki);
    m_kd.store(kd);
  }

  auto update(float const error) noexcept -> type::Position {
    float const kp = m_kp.load();
    float const ki = m_ki.load();
    float const kd = m_kd.load();
    float const last_error = m_last_error.load();

    float const p_term = kp * error;
    float const next_integral = std::clamp(m_integral.load() + ki * error * m_dt, m_min, m_max);
    m_integral.store(next_integral);

    float const d_term = kd * ((error - last_error) / m_dt);
    m_last_error.store(error);

    float const output = p_term + next_integral + d_term;
    float const rounded_output = std::roundf(output);

    return type::Position{static_cast<std::int32_t>(rounded_output)};
  }

  auto reset_to(float const base_value, float const initial_error = 0.0f) noexcept -> void {
    m_integral.store(std::clamp(base_value, m_min, m_max));
    m_last_error.store(initial_error);
  }
};

class ControllerCruise {
  static constexpr float dt_ecu{0.1f};
  static constexpr type::Position position_min{type::Position::value_min};
  static constexpr type::Position position_max{type::Position::value_max};

  common::AtomicContainer<bool> m_is_active{false};
  common::AtomicContainer<type::Speed> m_target_speed{0};
  common::AtomicContainer<type::Position> m_cruise_position{0};

  bool m_was_active_critical{false};
  PidController m_pid{dt_ecu, -position_max.get(), position_max.get()};
  type::Position m_last_position{type::Position::value_min};

 public:
  constexpr ControllerCruise() noexcept = default;

  auto calculate_pid_target(type::Speed const current_speed, type::Control const& control) noexcept -> void {
    m_pid.set_coefficients(control.cruise.p, control.cruise.i, control.cruise.d);

    float const error = m_target_speed.load().as<float>() - current_speed.as<float>();

    // ✅ Исправлено: Удален лишний аргумент false, так как PidController::update теперь принимает только error
    m_cruise_position.store(m_pid.update(error));
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

    type::Position const driver_position =
        common::map_range(accelerator, control.accelerator.min, control.accelerator.max, control.servo.min, control.servo.max);

    auto const driver_position_f = driver_position.as<float>();

    if (!is_active) {
      m_was_active_critical = false;
      m_pid.reset_to(driver_position_f, 0.0f);
      m_last_position = driver_position;
      return driver_position;
    }

    if (!m_was_active_critical) {
      float const initial_error = m_target_speed.load().as<float>() - current_speed.as<float>();
      m_pid.reset_to(driver_position_f, initial_error);
      m_last_position = driver_position;
      m_was_active_critical = true;
    }

    type::Position const cruise_position = m_cruise_position.load();
    if (driver_position > cruise_position) {
      m_last_position = driver_position;
      return driver_position;
    }

    type::Position const limiter = control.cruise.limiter;
    type::Position const cruise_position_minimal{m_last_position - limiter};
    type::Position const cruise_position_maximal{m_last_position + limiter};

    m_last_position = common::range(cruise_position, cruise_position_minimal, cruise_position_maximal);

    return m_last_position;
  }

  auto set_active(bool const active) noexcept -> void { m_is_active.store(active); }
  auto set_target_speed(type::Speed const speed) noexcept -> void { m_target_speed.store(speed); }

  [[nodiscard]] auto is_active() const noexcept -> bool { return m_is_active.load(); }
  [[nodiscard]] auto get_target_speed() const noexcept -> type::Speed { return m_target_speed.load(); }
};
