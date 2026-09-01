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
  static constexpr float speed_diff_f{speed_diff.as<float>()};
  static constexpr float regulator_value_min{type::Position::value_min};
  static constexpr float regulator_value_max{type::Position::value_max};

  common::PidRegulator<regulator_value_min, regulator_value_max> m_speed_regulator{
      common::PidCoefficients{
          .kp = 2.0f,
          .ki = 0.5f,
          .kd = 0.1f,
      },
      0.01f,
  };

 public:
  [[nodiscard]] auto calculate_servo_position(type::Position const accelerator_position,
                                              type::Speed const current_speed,
                                              type::Speed const target_speed,
                                              type::Control const& control) noexcept -> type::Position {
    type::Position driver_proposal = control.servo_min;

    if (auto const input_range = static_cast<float>(control.accelerator_max.as<int>() - control.accelerator_min.as<int>()); input_range > 0) {
      type::Position const clamped_input = std::clamp(accelerator_position.value, control.accelerator_min.value, control.accelerator_max.value);
      float const ratio = (clamped_input - control.accelerator_min).as<float>() / input_range;
      auto const output_range = (control.servo_max - control.servo_min).as<float>();
      float const interpolated = ratio * output_range + control.servo_min.as<float>();

      driver_proposal = type::Position{static_cast<std::int32_t>(std::roundf(interpolated))};
    }

    if (target_speed.value < 60) {
      m_speed_regulator.reset();
      return driver_proposal;
    }

    float const pid_value = m_speed_regulator.calculate(target_speed.value, current_speed.value);

    type::Position target_servo_position = control.servo_min;
    bool freeze_integral = false;

    if (type::Position const pid_servo_proposal{static_cast<std::int32_t>(std::roundf(pid_value))}; driver_proposal >= pid_servo_proposal) {
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
