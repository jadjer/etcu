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

#include <algorithm>
#include <cmath>

#include "common/map_range.hpp"
#include "common/pid_regulator.hpp"
#include "type/type.hpp"

class ControllerLogic {
  static constexpr float dt{0.01f};
  static constexpr float servo_min{0.0f};
  static constexpr float servo_max{1000.0f};
  static constexpr float max_change{120.0f};

  common::PidController m_pid{3.0f, 0.6f, 0.005f, dt, servo_min, servo_max};
  float m_last_servo{0.0f};
  bool m_was_active{false};

 public:
  [[nodiscard]] auto calculate_servo_position(type::Position const accelerator,
                                              type::Speed const current_speed,
                                              type::Speed const target_speed,
                                              bool const is_cruise_active,
                                              type::Control const& control) noexcept -> type::Position {
    type::Position const driver_pos_raw =
        common::map_range(accelerator, control.accelerator.min, control.accelerator.max, control.servo.min, control.servo.max);

    auto const driver_pos = driver_pos_raw.as<float>();

    if (!is_cruise_active) {
      m_pid.reset_to(driver_pos);
      m_last_servo = driver_pos;
      m_was_active = false;
      return driver_pos_raw;
    }

    float const error = target_speed.as<float>() - current_speed.as<float>();
    if (!m_was_active) {
      m_pid.reset_to(driver_pos, error);
      m_last_servo = driver_pos;
      m_was_active = true;
    }

    float const cruise_pos = m_pid.calculate(error);
    bool const driver_override = driver_pos >= cruise_pos;

    m_pid.update(error, driver_override);

    float const target_pos = std::clamp(std::max(driver_pos, cruise_pos), m_last_servo - max_change, m_last_servo + max_change);
    m_last_servo = std::clamp(target_pos, servo_min, servo_max);

    return type::Position{static_cast<std::int32_t>(std::roundf(m_last_servo))};
  }
};
