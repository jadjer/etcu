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

#include "common/atomic_container.hpp"
#include "common/range.hpp"
#include "type/control.hpp"
#include "type/type.hpp"

class PidController {
  float const m_dt;
  float const m_min{-250.0f};
  float const m_max{250.0f};

  common::AtomicContainer<float> m_kp{0.0f};
  common::AtomicContainer<float> m_ki{0.0f};
  common::AtomicContainer<float> m_kd{0.0f};
  common::AtomicContainer<float> m_integral{0.0f};
  common::AtomicContainer<float> m_last_error{0.0f};

 public:
  constexpr explicit PidController(float const dt) noexcept : m_dt{dt} {}

  constexpr PidController() noexcept = delete;
  PidController(PidController const&) noexcept = delete;
  auto operator=(PidController const&) noexcept -> PidController& = delete;
  PidController(PidController&&) noexcept = delete;
  auto operator=(PidController&&) noexcept -> PidController& = delete;
  constexpr ~PidController() noexcept = default;

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

    return type::Position{static_cast<std::int32_t>(output)};
  }

  auto reset_to(float const base_value, float const initial_error = 0.0f) noexcept -> void {
    m_integral.store(std::clamp(base_value, m_min, m_max));
    m_last_error.store(initial_error);
  }
};

class ControllerCruise {
  static constexpr float dt_ecu{0.1f};                   // 100 мс опрос K-Line
  static constexpr float fade_duration_s{1.0f};          // Плавный сброс за 1 сек
  static constexpr float critical_task_period_s{0.01f};  // 10 мс такт CriticalTask
  static constexpr float ticks_per_second{fade_duration_s / critical_task_period_s};
  static constexpr type::Position pid_min_position{0};
  static constexpr type::Position pid_max_position{1000};

  // Межъядерный интерфейс обмена данными (Core 0 <-> Core 1)
  struct InterCoreInterface {
    common::AtomicContainer<bool> is_active{false};
    common::AtomicContainer<bool> is_fading{false};
    common::AtomicContainer<type::Speed> target_speed{0};
    common::AtomicContainer<type::Position> cruise_position{0};
  };

  // Состояние потока ECUTask (Core 0)
  struct EcuTaskState {
    PidController pid{dt_ecu};
    float filtered_speed{0.0f};
  };

  // Состояние потока CriticalTask (Core 1)
  struct CriticalTaskState {
    bool was_active_critical{false};
    float fade_position_f{0.0f};
    type::Position last_position{type::Position::value_min};
  };

  InterCoreInterface m_ipc{};
  EcuTaskState m_ecu_state{};
  CriticalTaskState m_critical_state{};

 public:
  constexpr ControllerCruise() noexcept = default;
  ControllerCruise(ControllerCruise const&) noexcept = delete;
  auto operator=(ControllerCruise const&) noexcept -> ControllerCruise& = delete;
  ControllerCruise(ControllerCruise&&) noexcept = delete;
  auto operator=(ControllerCruise&&) noexcept -> ControllerCruise& = delete;
  constexpr ~ControllerCruise() noexcept = default;

  // 🔄 ВЫЗЫВАЕТСЯ НА CORE 0 (ECUTask - 100 мс)
  auto calculate_pid_target(type::Speed const current_speed, type::Control const& control) noexcept -> void {
    m_ecu_state.pid.set_coefficients(control.cruise.p, control.cruise.i, control.cruise.d);

    // ФНЧ для подавления дискретного шума K-Line скорости
    if (m_ecu_state.filtered_speed == 0.0f) {
      m_ecu_state.filtered_speed = current_speed.as<float>();
    } else {
      m_ecu_state.filtered_speed = current_speed.as<float>() * 0.25f + m_ecu_state.filtered_speed * 0.75f;
    }

    if (!m_ipc.is_active.load() && !m_ipc.is_fading.load()) {
      return;
    }

    float const error = m_ipc.target_speed.load().as<float>() - m_ecu_state.filtered_speed;
    m_ipc.cruise_position.store(m_ecu_state.pid.update(error));
  }

  // 🔄 ВЫЗЫВАЕТСЯ НА CORE 1 (CriticalTask - 10 мс)
  [[nodiscard]] auto generate_throttle_position(type::Position const accelerator,
                                             bool const safety_active,
                                             type::Control const& control,
                                             type::Speed const current_speed) noexcept -> type::Position {
    type::Position const driver_position =
        common::map_range(accelerator, control.accelerator.min, control.accelerator.max, control.servo.min, control.servo.max);

    // 🔴 СЦЕНАРИЙ А: Экстренное прерывание
    if (safety_active) {
      m_ipc.is_active.store(false);
      m_ipc.is_fading.store(false);
      m_critical_state.was_active_critical = false;
      m_critical_state.last_position = driver_position;
      return driver_position;
    }

    bool const is_cruise_active = m_ipc.is_active.load();
    bool const is_fade_active = m_ipc.is_fading.load();

    // 🟡 СЦЕНАРИЙ Б: Плавный выход за 1 секунду
    if (!is_cruise_active) {
      m_critical_state.was_active_critical = false;

      if (!is_fade_active) {
        if (m_critical_state.last_position > driver_position) {
          m_ipc.is_fading.store(true);
          m_critical_state.fade_position_f = m_critical_state.last_position.as<float>();
        } else {
          m_critical_state.last_position = driver_position;
          return driver_position;
        }
      }

      if (is_fade_active) {
        float const fade_step_per_tick = static_cast<float>(control.servo.max.get()) / ticks_per_second;
        m_critical_state.fade_position_f -= fade_step_per_tick;

        type::Position const current_fade_position{static_cast<std::int32_t>(m_critical_state.fade_position_f)};

        if (current_fade_position <= driver_position || m_critical_state.fade_position_f <= 0.0f) {
          m_ipc.is_fading.store(false);
          m_critical_state.last_position = driver_position;
          return driver_position;
        }

        m_critical_state.last_position = current_fade_position;
        return m_critical_state.last_position;
      }

      m_critical_state.last_position = driver_position;
      return driver_position;
    }

    // 🟢 СЦЕНАРИЙ В: Штатный круиз
    if (is_fade_active) {
      m_ipc.is_fading.store(false);
    }

    if (!m_critical_state.was_active_critical) {
      float const initial_error = m_ipc.target_speed.load().as<float>() - current_speed.as<float>();

      type::Position const driver_pos_normalized = common::map_range(driver_position, control.servo.min, control.servo.max, pid_min_position, pid_max_position);

      m_ecu_state.pid.reset_to(driver_pos_normalized.as<float>(), initial_error);
      m_critical_state.last_position = driver_position;
      m_critical_state.was_active_critical = true;
    }

    type::Position const cruise_position =
        common::map_range(m_ipc.cruise_position.load(), pid_min_position, pid_max_position, control.servo.min, control.servo.max);

    if (driver_position > cruise_position) {
      m_critical_state.last_position = driver_position;
      return driver_position;
    }

    std::int32_t scaled_limiter = control.cruise.limiter.get() / 10;
    if (scaled_limiter < 1) {
      scaled_limiter = 1;
    }

    // Асимметрия: с горы закрываем в 5 раз быстрее для удержания веса Варадеро
    type::Position const cruise_position_minimal{m_critical_state.last_position.get() - (scaled_limiter * 5)};
    type::Position const cruise_position_maximal{m_critical_state.last_position.get() + scaled_limiter};

    m_critical_state.last_position = common::range(cruise_position, cruise_position_minimal, cruise_position_maximal);

    return m_critical_state.last_position;
  }

  auto set_active(bool const active) noexcept -> void { m_ipc.is_active.store(active); }
  auto set_target_speed(type::Speed const speed) noexcept -> void { m_ipc.target_speed.store(speed); }

  [[nodiscard]] auto is_active() const noexcept -> bool { return m_ipc.is_active.load(); }
  [[nodiscard]] auto get_target_speed() const noexcept -> type::Speed { return m_ipc.target_speed.load(); }
};
