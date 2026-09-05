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

enum class CruiseState { Idle, CheckingStability, Active };

struct ControllerLogicResult {
  type::Position servo_position;
  type::Speed new_target_speed{0};
  bool is_speed_changed{false};
};

class ControllerLogic {
  static constexpr type::Speed speed_start_fade_kmh{60};
  static constexpr float loop_dt{0.1f};
  static constexpr float regulator_value_min{type::Position::value_min};
  static constexpr float regulator_value_max{type::Position::value_max};

  common::PidRegulator<regulator_value_min, regulator_value_max> m_speed_regulator{
      common::PidCoefficients{.kp = 2.0f, .ki = 0.5f, .kd = 0.1f},
      loop_dt,
  };

  CruiseState m_cruise_state{CruiseState::Idle};
  type::Speed m_base_speed{0};
  std::uint32_t m_stability_ticks{0};

  [[nodiscard]] static auto interpolate_accelerator(type::Position const accelerator, type::Control const& control) noexcept -> type::Position {
    auto const input_range = control.accelerator.max.as<float>() - control.accelerator.min.as<float>();

    if (input_range <= 0.0f) {
      return control.servo.min;
    }

    float const clamped_input = std::clamp(accelerator.as<float>(), control.accelerator.min.as<float>(), control.accelerator.max.as<float>());
    float const ratio = (clamped_input - control.accelerator.min.as<float>()) / input_range;
    float const output_range = control.servo.max.as<float>() - control.servo.min.as<float>();

    return type::Position{static_cast<std::int32_t>(std::roundf(ratio * output_range + control.servo.min.as<float>()))};
  }

  [[nodiscard]] auto handle_safety_reset(type::Position const driver_position, type::Speed const target_speed) noexcept -> ControllerLogicResult {
    if (m_cruise_state == CruiseState::Active || target_speed > 0) {
      m_speed_regulator.reset();
      m_cruise_state = CruiseState::Idle;

      return {.servo_position = driver_position, .new_target_speed = type::Speed{0}, .is_speed_changed = true};
    }

    m_cruise_state = CruiseState::Idle;

    return {.servo_position = driver_position, .new_target_speed = type::Speed{0}, .is_speed_changed = false};
  }

  auto process_auto_set_state(type::Speed const current_speed,
                              type::Speed const target_speed,
                              type::Control const& control,
                              ControllerLogicResult& result) noexcept -> void {
    switch (m_cruise_state) {
      case CruiseState::Idle:
        if (control.cruise.enabled && target_speed == 0 && current_speed > control.cruise.threshold_kmh) {
          m_base_speed = current_speed;
          m_stability_ticks = 0;
          m_cruise_state = CruiseState::CheckingStability;
          ESP_LOGI("LOG", "CheckingStability");
        }
        break;

      case CruiseState::CheckingStability:
        if (!control.cruise.enabled) {
          m_cruise_state = CruiseState::Idle;
          ESP_LOGI("LOG", "Idle");
          break;
        }

        if (std::abs(current_speed.get() - m_base_speed.get()) <= control.cruise.tolerance_kmh) {
          m_stability_ticks++;

          if (float const delay_seconds = control.cruise.delay_sec; m_stability_ticks >= static_cast<std::uint32_t>(delay_seconds / loop_dt)) {
            result.new_target_speed = current_speed;
            result.is_speed_changed = true;
            m_speed_regulator.reset();
            m_cruise_state = CruiseState::Active;
            ESP_LOGI("LOG", "Active");
          }

        } else {
          m_base_speed = current_speed;
          m_stability_ticks = 0;

          if (current_speed.get() <= control.cruise.threshold_kmh.get()) {
            m_cruise_state = CruiseState::Idle;
            ESP_LOGI("LOG", "Idle");
          }
        }

        break;

      case CruiseState::Active:
        if (current_speed.get() < speed_start_fade_kmh.get()) {
          m_speed_regulator.reset();
          result.new_target_speed = type::Speed{0};
          result.is_speed_changed = true;
          m_cruise_state = CruiseState::Idle;
          ESP_LOGI("LOG", "Idle");
        }

        break;
    }
  }

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
                                              bool const safety_active) noexcept -> ControllerLogicResult {
    ControllerLogicResult result{};

    result.servo_position = interpolate_accelerator(accelerator_position, control);

    if (safety_active) {
      return handle_safety_reset(result.servo_position, target_speed);
    }

    process_auto_set_state(current_speed, target_speed, control, result);

    if (m_cruise_state == CruiseState::Active) {
      result.servo_position = calculate_cruise_control(result.servo_position, current_speed, target_speed);
    }

    return result;
  }
};
