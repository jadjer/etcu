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

#include "common/atomic_container.hpp"
#include "common/range.hpp"
#include "type/control.hpp"
#include "type/type.hpp"

class PIDController {
  float const m_dt;
  float m_kp{0.0f};
  float m_ki{0.0f};
  float m_kd{0.0f};
  float m_integral{0.0f};
  float m_last_error{0.0f};

 public:
  constexpr explicit PIDController(float const dt) noexcept : m_dt{dt} {}

  constexpr PIDController() noexcept = delete;
  PIDController(PIDController const&) noexcept = delete;
  auto operator=(PIDController const&) noexcept -> PIDController& = delete;
  PIDController(PIDController&&) noexcept = delete;
  auto operator=(PIDController&&) noexcept -> PIDController& = delete;
  constexpr ~PIDController() noexcept = default;

  auto set_coefficients(float const kp, float const ki, float const kd) noexcept -> void {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
  }

  auto update(float const error, float const integral_limit_min, float const integral_limit_max) noexcept -> type::Position {
    float const p_term = m_kp * error;
    float const integral = m_integral + m_ki * error * m_dt;
    float const integral_limit = std::clamp(integral, integral_limit_min, integral_limit_max);

    m_integral = integral_limit;

    float const d_term = m_kd * ((error - m_last_error) / m_dt);
    m_last_error = error;

    float const output = p_term + integral_limit + d_term;
    return type::Position{output};
  }

  auto reset_to(float const base_value, float const integral_limit_min, float const integral_limit_max, float const initial_error = 0.0f) noexcept -> void {
    m_integral = std::clamp(base_value, integral_limit_min, integral_limit_max);
    m_last_error = initial_error;
  }
};

// =============================================================================
// КЛАСС УПРАВЛЕНИЯ КРУИЗ-КОНТРОЛЕМ (Строго в типах Position и Speed)
// =============================================================================
class ControllerCruise {
  static constexpr float critical_task_period_s{0.01f};  // 10 мс такт CriticalTask (Core 1)

  // Межпоточный интерфейс (Связь с Core 0)
  common::AtomicContainer<bool> m_is_active{false};
  common::AtomicContainer<type::Speed> m_target_speed{0};

  // Переменные внутреннего состояния контура управления Core 1
  bool m_is_fading{false};
  bool m_was_active{false};
  float m_filtered_speed{0.0f};
  type::Position m_last_position{type::Position::value_min};
  PIDController m_pid{critical_task_period_s};

  // 🔴 Линейный блок А: Экстренный аварийный сброс заслонки
  auto process_safety_cutoff(type::Position const driver_position) noexcept -> type::Position {
    m_is_active.store(false);
    m_is_fading = false;
    m_was_active = false;
    m_filtered_speed = 0.0f;
    m_last_position = driver_position;
    return driver_position;
  }

  // 🟡 Линейный блок Б: Плавный линейный спуск газа за 1 секунду
  auto process_fade_out(type::Position const driver_position, type::Control const& control) noexcept -> type::Position {
    m_was_active = false;
    m_filtered_speed = 0.0f;

    // Шаг 1: Ранний возврат, если спуск завершен или ручка водителя открыта сильнее
    if (!m_is_fading && m_last_position <= driver_position) {
      m_last_position = driver_position;
      return driver_position;
    }

    // Шаг 2: Взведение триггера при первом такте отключения
    if (!m_is_fading) {
      m_is_fading = true;
    }

    // Шаг 3: Линейный спуск на основе физического максимума сервопривода
    float const ticks_per_second = control.cruise.fade_duration / critical_task_period_s;
    float const fade_step = control.servo_max.as<float>() / ticks_per_second;
    type::Position const next_position = m_last_position - fade_step;

    // Шаг 4: Проверка условия завершения режима fade по реальной ручке водителя
    if (next_position <= driver_position || next_position <= type::Position::value_min) {
      m_is_fading = false;
      m_last_position = driver_position;
      return driver_position;
    }

    m_last_position = next_position;
    return m_last_position;
  }

  // 🟢 Линейный блок В: Активное ПИД-удержание целевой скорости
  auto process_active_cruise(type::Position const driver_position, type::Control const& control, type::Speed const current_speed) noexcept -> type::Position {
    m_is_fading = false;
    m_pid.set_coefficients(control.cruise.p, control.cruise.i, control.cruise.d);

    // Фильтрация низких частот скорости (ФНЧ) в типе type::Speed
    if (m_filtered_speed == 0.0f) {
      m_filtered_speed = current_speed.as<float>();
    } else {
      float const alpha = control.cruise.filter_alpha;
      m_filtered_speed = current_speed.as<float>() * alpha + m_filtered_speed * (1.0f - alpha);
    }

    type::Speed const target_speed = m_target_speed.load();

    // Синхронизация ПИД-контура при первом запуске
    if (!m_was_active) {
      float const initial_error = target_speed.as<float>() - current_speed.as<float>();
      m_pid.reset_to(driver_position.as<float>(), control.cruise.integral_min, control.cruise.integral_max, initial_error);
      m_last_position = driver_position;
      m_was_active = true;
    }

    // Расчет целевой физической позиции ПИД
    float const error = target_speed.as<float>() - m_filtered_speed;
    type::Position const cruise_position_physical = m_pid.update(error, control.cruise.integral_min, control.cruise.integral_max);

    // Приоритет человека (Обгон): если ручка водителя открыта сильнее — выходим по раннему возврату
    if (driver_position > cruise_position_physical) {
      m_last_position = driver_position;
      return driver_position;
    }

    // Ограничение скользящего окна безопасности
    type::Position const cruise_position_minimal{m_last_position - control.cruise.limiter_left};
    type::Position const cruise_position_maximal{m_last_position + control.cruise.limiter_right};

    m_last_position = common::range(cruise_position_physical, cruise_position_minimal, cruise_position_maximal);

    return m_last_position;
  }

 public:
  constexpr ControllerCruise() noexcept = default;
  ControllerCruise(ControllerCruise const&) noexcept = delete;
  auto operator=(ControllerCruise const&) noexcept -> ControllerCruise& = delete;
  ControllerCruise(ControllerCruise&&) noexcept = delete;
  auto operator=(ControllerCruise&&) noexcept -> ControllerCruise& = delete;
  constexpr ~ControllerCruise() noexcept = default;

  // 🔄 ИДЕАЛЬНО ЛИНЕЙНЫЙ ДИСПЕТЧЕР-КАЛИБРОВЩИК (Core 1 - CriticalTask 10 мс)
  [[nodiscard]] auto generate_throttle_position(type::Position const accelerator,
                                                bool const safety_active,
                                                type::Control const& control,
                                                type::Speed const current_speed) noexcept -> type::Position {
    // 1. Входная калибровка ручки: перевод сырого положения акселератора в физический тип Position
    type::Position const driver_position{accelerator};

    // 2. Линейный сквозной выбор сценария работы на основе ранних возвратов (Early Returns)
    if (safety_active) {
      type::Position const safety_target = process_safety_cutoff(driver_position);
      return common::map_range(safety_target, control.accelerator_min, control.accelerator_max, control.servo_min, control.servo_max);
    }

    if (!m_is_active.load()) {
      type::Position const fade_target = process_fade_out(driver_position, control);
      return common::map_range(fade_target, control.accelerator_min, control.accelerator_max, control.servo_min, control.servo_max);
    }

    type::Position const cruise_target = process_active_cruise(driver_position, control, current_speed);

    // 3. ✅ ЕДИНАЯ ДЕНОРМАЛИЗАЦИЯ: Перевод физического значения Position в ход сервопривода строго в самом конце
    return common::map_range(cruise_target, control.accelerator_min, control.accelerator_max, control.servo_min, control.servo_max);
  }

  // Сервисные методы сопряжения с ядром Core 0
  auto set_active(bool const active) noexcept -> void { m_is_active.store(active); }
  auto set_target_speed(type::Speed const speed) noexcept -> void { m_target_speed.store(speed); }

  [[nodiscard]] auto is_active() const noexcept -> bool { return m_is_active.load(); }
  [[nodiscard]] auto get_target_speed() const noexcept -> type::Speed { return m_target_speed.load(); }
};
