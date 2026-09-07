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

#include <cmath>

#include "common/map_range.hpp"
#include "common/pid_regulator.hpp"
#include "type/type.hpp"

class ControllerLogic {
  static constexpr float regulator_value_min{type::Position::value_min};
  static constexpr float regulator_value_max{type::Position::value_max};

  common::PidRegulator<regulator_value_min, regulator_value_max> m_speed_regulator{
      common::PidCoefficients{.kp = 1.5f, .ki = 0.2f, .kd = 0.02f},
      0.01f,
  };

  [[nodiscard]] auto calculate_cruise_control(type::Position const driver_proposal, type::Speed const current_speed, type::Speed const target_speed) noexcept
      -> type::Position {
    float const pid_value = m_speed_regulator.calculate(target_speed.get(), current_speed.get());
    auto const pid_servo_proposal = type::Position{static_cast<std::int32_t>(std::roundf(pid_value))};

    bool const driver_override = driver_proposal >= pid_servo_proposal;
    m_speed_regulator.update(target_speed.as<float>(), current_speed.as<float>(), driver_override);

    return driver_override ? driver_proposal : pid_servo_proposal;
  }

 public:
  [[nodiscard]] auto calculate_servo_position(type::Position const accelerator_position,
                                              type::Speed const current_speed,
                                              type::Speed const target_speed,
                                              type::Control const& control,
                                              bool const safety_active) noexcept -> type::Position {
    auto const driver_position =
        common::map_range(accelerator_position, control.accelerator.min, control.accelerator.max, control.servo.min, control.servo.max);

    if (safety_active) {
      m_speed_regulator.reset();
      return driver_position;
    }

    if (target_speed > 0 && current_speed >= control.cruise.threshold) {
      return calculate_cruise_control(driver_position, current_speed, target_speed);
    }

    return driver_position;
  }
};
