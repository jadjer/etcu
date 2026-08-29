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

class Logic {
  static constexpr type::Speed speed_start_fade_kmh{60};
  static constexpr type::Speed speed_end_fade_kmh{100};
  static constexpr type::Speed speed_diff{speed_end_fade_kmh - speed_start_fade_kmh};
  static constexpr float speed_diff_f = speed_diff.as<float>();

  common::PidRegulator<static_cast<float>(type::Position::value_min), static_cast<float>(type::Position::value_max)> m_speed_regulator{common::PidCoefficients{
                                                                                                                                           .kp = 2.0f,
                                                                                                                                           .ki = 0.5f,
                                                                                                                                           .kd = 0.1f,
                                                                                                                                       },
                                                                                                                                       0.01f};

 public:
  [[nodiscard]] auto calculate_servo_position(type::Position const accelerator_position,
                                              type::Position const accelerator_min,
                                              type::Position const accelerator_max,
                                              type::Position const servo_min,
                                              type::Position const servo_max,
                                              type::Speed const current_speed,
                                              type::Speed const target_speed) noexcept -> type::Position {
    // 1. Диапазоны (благодаря оператору '-' возвращают чистый int32_t)
    std::int32_t const input_range = accelerator_max.value - accelerator_min.value;
    std::int32_t const output_range = servo_max.value - servo_min.value;

    type::Position driver_proposal = servo_min;

    if (input_range > 0) {
      // accelerator_position.value гарантированно валиден, но если он вне калиброванных
      // min/max (например, педаль не до конца отпущена), делаем clamp
      std::int32_t const clamped_input = std::clamp(accelerator_position.value, accelerator_min.value, accelerator_max.value);

      // Интерполяция
      float const ratio = static_cast<float>(clamped_input - accelerator_min.value) / static_cast<float>(input_range);
      float const interpolated = (ratio * static_cast<float>(output_range)) + static_cast<float>(servo_min.value);

      // Конструктор type::Position сам сделает clamp к физическим границам типа, если мы вышли за них
      driver_proposal = type::Position{static_cast<std::int32_t>(std::roundf(interpolated))};
    }

    // 2. Логика круиз-контроля
    if (target_speed.value < 60) {
      m_speed_regulator.reset();
      return driver_proposal;
    }

    float const pid_value = m_speed_regulator.calculate(target_speed.value, current_speed.value);

    type::Position const pid_servo_proposal{static_cast<std::int32_t>(std::roundf(pid_value))};

    type::Position target_servo_position = servo_min;
    bool freeze_integral = false;

    if (driver_proposal >= pid_servo_proposal) {
      target_servo_position = driver_proposal;
      freeze_integral = true;
    } else {
      target_servo_position = pid_servo_proposal;
      freeze_integral = false;
    }

    m_speed_regulator.update(target_speed.value, current_speed.value, freeze_integral);

    return target_servo_position;
  }
};
