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

#include "common/pid_regulator.hpp"
#include "type/type.hpp"

template <std::size_t LutSize>
consteval auto generate_linear_lut() -> std::array<type::Position, LutSize> {
  static constexpr std::size_t lut_size{LutSize};
  static constexpr auto max_servo_output = static_cast<float>(type::Position::max_value);

  std::array<type::Position, lut_size> lut{};

  for (std::size_t i = 0; i < lut_size; ++i) {
    float const progress = static_cast<float>(i) / static_cast<float>(lut_size - 1);
    float const calculated = progress * max_servo_output;

    lut[i] = type::Position{static_cast<type::primitive::Position>(calculated)};
  }

  return lut;
}

class Logic {
  static constexpr std::size_t lut_size{type::Position::max_value + 1};
  static constexpr std::array<type::Position, lut_size> servo_lut_100pct{generate_linear_lut<lut_size>()};

  static constexpr type::Speed speed_start_fade_kmh{60};
  static constexpr type::Speed speed_end_fade_kmh{100};
  static constexpr type::Speed speed_diff{speed_end_fade_kmh - speed_start_fade_kmh};
  static constexpr float speed_diff_f = speed_diff.as<float>();

  common::PidRegulator<static_cast<float>(type::Position::min_value), static_cast<float>(type::Position::max_value)> m_speed_regulator{common::PidCoefficients{
                                                                                                                                           .kp = 2.0f,
                                                                                                                                           .ki = 0.5f,
                                                                                                                                           .kd = 0.1f,
                                                                                                                                       },
                                                                                                                                       0.01f};

 public:
  [[nodiscard]] auto calculate_servo_position(type::Position const accelerator_position,
                                              type::Position const accelerator_offset,
                                              type::Speed const current_speed,
                                              type::Speed const target_speed) noexcept -> type::Position {
    auto const index = accelerator_position + accelerator_offset;
    auto const raw_driver_proposal = servo_lut_100pct[index.value].as<float>();

    float scale_factor = 0.5f;  // По умолчанию 50%

    if (current_speed >= speed_end_fade_kmh) {
      scale_factor = 1.0f;
    } else if (current_speed > speed_start_fade_kmh) {
      float const speed_progress = (current_speed - speed_start_fade_kmh).as<float>() / speed_diff_f;
      scale_factor = 0.5f + speed_progress * 0.5f;
    }

    // 3. Масштабируем предложение водителя
    type::Position const driver_servo_proposal{static_cast<type::primitive::Position>(raw_driver_proposal * scale_factor)};

    // --- Дальнейшая логика ПИД-регулятора остается без изменений ---
    if (target_speed.value < 60) {
      m_speed_regulator.reset();
      return driver_servo_proposal;
    }

    float const pid_value = m_speed_regulator.calculate(target_speed.value, current_speed.value);
    type::Position const pid_servo_proposal{static_cast<type::primitive::Position>(pid_value)};

    type::Position target_servo_position{0};
    bool freeze_integral = false;

    if (driver_servo_proposal >= pid_servo_proposal) {
      target_servo_position = driver_servo_proposal;
      freeze_integral = true;
    } else {
      target_servo_position = pid_servo_proposal;
      freeze_integral = false;
    }

    m_speed_regulator.update(target_speed.value, current_speed.value, freeze_integral);

    return target_servo_position;
  }
};
