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

#include "algorithm"
#include "cmath"
#include "common/pid_regulator.hpp"
#include "type/type.hpp"

struct ControllerLogicResult {
  type::Position servo_position;
  type::Speed new_target_speed{0};
  bool is_speed_changed{false};
};

class ControllerLogic {
  enum class CruiseState {
    Idle,               // Обычная езда ногами/руками
    CheckingStability,  // Идет авто-замер стабильности скорости
    Active              // Авто-круиз зафиксирован и удерживает скорость
  };

  // Внутренние константы логики
  static constexpr type::Speed speed_start_fade_kmh{60};
  static constexpr float loop_dt{0.01f}; // Шаг критического цикла 10 мс (100 Гц)

  // Лимиты для регулятора положения
  static constexpr float regulator_value_min{type::Position::value_min};
  static constexpr float regulator_value_max{type::Position::value_max};

  // PID-регулятор скорости
  common::PidRegulator<regulator_value_min, regulator_value_max> m_speed_regulator{
      common::PidCoefficients{
          .kp = 2.0f,
          .ki = 0.5f,
          .kd = 0.1f,
      },
      loop_dt,
  };

  // Переменные состояния автомата
  CruiseState m_cruise_state{CruiseState::Idle};
  type::Speed m_base_speed{0};
  std::uint32_t m_stability_ticks{0};

 public:
  [[nodiscard]] auto calculate_servo_position(type::Position const accelerator_position,
                                              type::Speed const current_speed,
                                              type::Speed const target_speed, // Снова константный!
                                              type::Control const& control,
                                              bool const safety_active) noexcept -> ControllerLogicResult {

    ControllerLogicResult result{};

    // 1. Расчет желаемой позиции водителя (интерполяция)
    type::Position driver_proposal = control.servo.min;
    if (auto const input_range = static_cast<float>(control.accelerator.max.get() - control.accelerator.min.get()); input_range > 0) {
      type::Position const clamped_input = std::clamp(accelerator_position.get(), control.accelerator.min.get(), control.accelerator.max.get());
      float const ratio = static_cast<float>(clamped_input.get() - control.accelerator.min.get()) / input_range;
      auto const output_range = static_cast<float>(control.servo.max.get() - control.servo.min.get());
      float const interpolated = ratio * output_range + static_cast<float>(control.servo.min.get());

      driver_proposal = type::Position{static_cast<std::int32_t>(std::roundf(interpolated))};
    }
    result.servo_position = driver_proposal;

    // 2. Мгновенный сброс по тормозу или нейтрали
    if (safety_active) {
      if (m_cruise_state == CruiseState::Active || target_speed.get() > 0) {
        m_speed_regulator.reset();
        result.new_target_speed = type::Speed{0};
        result.is_speed_changed = true; // Требуем от контроллера обновить атомик
      }
      m_cruise_state = CruiseState::Idle;
      return result;
    }

    // Извлекаем калибровки из подструктуры auto_set
    bool const auto_set_enabled = control.auto_set.enabled;
    type::Speed const auto_set_threshold{control.auto_set.threshold};
    type::Speed const auto_set_tolerance{control.auto_set.tolerance};

    // Динамический пересчет задержки (delay из ms в секунды и в такты)
    float const delay_seconds = static_cast<float>(control.auto_set.delay) / 1000.0f;
    auto const target_ticks = static_cast<std::uint32_t>(delay_seconds / loop_dt);

    // 3. Конечный автомат автоматического подхвата скорости
    switch (m_cruise_state) {
      case CruiseState::Idle: {
        if (auto_set_enabled && target_speed.get() == 0 && current_speed.get() > auto_set_threshold.get()) {
          m_base_speed = current_speed;
          m_stability_ticks = 0;
          m_cruise_state = CruiseState::CheckingStability;
        }
        break;
      }

      case CruiseState::CheckingStability: {
        if (!auto_set_enabled) {
          m_cruise_state = CruiseState::Idle;
          break;
        }

        if (current_speed.get() >= (m_base_speed.get() - auto_set_tolerance.get()) &&
            current_speed.get() <= (m_base_speed.get() + auto_set_tolerance.get())) {

          m_stability_ticks++;
          if (m_stability_ticks >= target_ticks) {
            result.new_target_speed = current_speed; // АВТО-СЕТ: фиксация скорости
            result.is_speed_changed = true;          // Требуем от контроллера записать её в атомик
            m_speed_regulator.reset();
            m_cruise_state = CruiseState::Active;
          }
        } else {
          m_base_speed = current_speed;
          m_stability_ticks = 0;

          if (current_speed.get() <= auto_set_threshold.get()) {
            m_cruise_state = CruiseState::Idle;
          }
        }
        break;
      }

      case CruiseState::Active: {
        if (current_speed.get() < speed_start_fade_kmh.get()) {
          m_speed_regulator.reset();
          result.new_target_speed = type::Speed{0}; // Выход из окна вниз: авто-сброс
          result.is_speed_changed = true;
          m_cruise_state = CruiseState::Idle;
        }
        break;
      }
    }

    // 4. Расчет PID регулятора скорости
    if (m_cruise_state != CruiseState::Active) {
      return result;
    }

    float const pid_value = m_speed_regulator.calculate(target_speed.get(), current_speed.get());
    bool freeze_integral = false;

    // Приоритет ручки газа водителя поверх автоматики
    if (type::Position const pid_servo_proposal{static_cast<std::int32_t>(std::roundf(pid_value))}; driver_proposal.get() >= pid_servo_proposal.get()) {
      result.servo_position = driver_proposal;
      freeze_integral = true;
    } else {
      result.servo_position = pid_servo_proposal;
      freeze_integral = false;
    }

    m_speed_regulator.update(target_speed.get(), current_speed.get(), freeze_integral);

    return result;
  }
};
