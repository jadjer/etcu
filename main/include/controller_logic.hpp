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

template <type::primitive::Size N>
consteval auto generate_flexible_lut() -> std::array<type::Position, N> {
  static constexpr type::Position SERVO_SWITCH{static_cast<type::primitive::Position>(static_cast<float>(type::Position::MaxValue) * 0.3f)};
  static constexpr float PEDAL_SWITCH{0.5f};
  static constexpr float PARABOLA_K{1.0f / (PEDAL_SWITCH * PEDAL_SWITCH)};

  std::array<type::Position, N> lut;

  for (type::primitive::Size i = 0; i < N; ++i) {
    float const progress = static_cast<float>(i) / static_cast<float>(N - 1);

    if (progress <= PEDAL_SWITCH) {
      auto const calculated = PARABOLA_K * static_cast<float>(SERVO_SWITCH.value) * progress * progress;

      lut[i] = static_cast<type::primitive::Position>(calculated);

    } else {
      auto const segment_progress = (progress - PEDAL_SWITCH) / (1.0f - PEDAL_SWITCH);
      constexpr auto delta_servo = static_cast<float>(type::Position::MaxValue - SERVO_SWITCH.value);
      auto const calculated = static_cast<float>(SERVO_SWITCH.value) + segment_progress * delta_servo;

      lut[i] = static_cast<type::primitive::Position>(calculated);
    }
  }

  return lut;
}

class Logic {
  static constexpr type::primitive::Size LUT_SIZE{static_cast<type::primitive::Size>(static_cast<float>(type::Position::MaxValue) * 0.3f)};
  static constexpr std::array<type::Position, LUT_SIZE> SERVO_LUT{generate_flexible_lut<LUT_SIZE>()};

  common::PidRegulator<static_cast<float>(type::Position::MinValue), static_cast<float>(type::Position::MaxValue)> m_speed_regulator{common::PidCoefficients{
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
    type::Position const accelerator_value{accelerator_position + accelerator_offset};
    type::Position const driver_servo_proposal{SERVO_LUT[accelerator_value.value]};

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
