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

#include <pid_ctrl.h>

#include "common/atomic_container.hpp"
#include "common/range.hpp"
#include "type/control.hpp"
#include "type/type.hpp"

class ControllerCruise {
  static constexpr float critical_task_period_s{0.01f};

  common::AtomicContainer<bool> m_is_active{false};
  common::AtomicContainer<bool> m_need_reset{false};
  common::AtomicContainer<bool> m_is_running{false};

  float m_filtered_speed{0.0f};
  type::Speed m_target_speed{0};
  type::Position m_base_throttle{type::Position::value_min};
  type::Position m_last_position{type::Position::value_min};

  pid_ctrl_block_handle_f_t m_pid_handle{nullptr};

  auto process_inactive_cruise(type::Position const driver_position) noexcept -> type::Position {
    m_is_running.store(false);
    m_filtered_speed = 0.0f;
    m_last_position = driver_position;

    if (m_pid_handle != nullptr) {
      pid_reset_ctrl_block(m_pid_handle);
    }

    return driver_position;
  }

  auto process_active_cruise(type::Position const driver_position, type::Control const& control, type::Speed const current_speed) noexcept -> type::Position {
    if (m_pid_handle == nullptr) [[unlikely]] {
      return driver_position;
    }

    bool const is_reset_requested = m_need_reset.load();
    if (is_reset_requested) {
      m_need_reset.store(false);
    }

    bool const is_running = m_is_running.load();

    if (!is_running || is_reset_requested) {
      if (is_reset_requested) {
        m_target_speed = current_speed;
        m_base_throttle = driver_position;
      }

      m_is_running.store(true);
      m_last_position = driver_position;

      pid_reset_ctrl_block(m_pid_handle);
    }

    pid_ctrl_parameter_f_t const pid_params = {
        .kp = control.cruise.p,
        .ki = control.cruise.i,
        .kd = control.cruise.d,
        .max_output = control.cruise.limiter_right.as<float>(),
        .min_output = -control.cruise.limiter_left.as<float>(),
        .max_integral = control.cruise.integral_max,
        .min_integral = control.cruise.integral_min,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
    };
    pid_update_parameters(m_pid_handle, &pid_params);

    if (m_filtered_speed == 0.0f) {
      m_filtered_speed = current_speed.as<float>();
    } else {
      float const alpha = control.cruise.filter_alpha;
      m_filtered_speed = current_speed.as<float>() * alpha + m_filtered_speed * (1.0f - alpha);
    }

    float const error = m_target_speed.as<float>() - m_filtered_speed;
    float pid_correction{0.0f};

    if (pid_compute(m_pid_handle, error, &pid_correction) != ESP_OK) [[unlikely]] {
      return driver_position;
    }

    type::Position const cruise_position = m_base_throttle + pid_correction;

    if (driver_position > cruise_position) {
      m_last_position = driver_position;
      return driver_position;
    }

    type::Position const cruise_position_minimal{m_last_position - control.cruise.limiter_left};
    type::Position const cruise_position_maximal{m_last_position + control.cruise.limiter_right};

    m_last_position = common::range(cruise_position, cruise_position_minimal, cruise_position_maximal);

    return m_last_position;
  }

 public:
  constexpr ControllerCruise() noexcept {
    static constexpr pid_ctrl_config_f_t pid_config{
        .init_param = {.kp = 0.0f,
                       .ki = 0.0f,
                       .kd = 0.0f,
                       .max_output = 0.0f,
                       .min_output = 0.0f,
                       .max_integral = 0.0f,
                       .min_integral = 0.0f,
                       .cal_type = PID_CAL_TYPE_POSITIONAL},
    };
    pid_new_control_block_f(&pid_config, &m_pid_handle);
  }

  ControllerCruise(ControllerCruise const&) noexcept = delete;
  auto operator=(ControllerCruise const&) noexcept -> ControllerCruise& = delete;
  ControllerCruise(ControllerCruise&&) noexcept = delete;
  auto operator=(ControllerCruise&&) noexcept -> ControllerCruise& = delete;

  ~ControllerCruise() noexcept {
    if (m_pid_handle != nullptr) {
      pid_del_control_block(m_pid_handle);
    }
  }

  [[nodiscard]] auto generate_throttle_position(type::Position const accelerator,
                                                bool const safety_active,
                                                type::Control const& control,
                                                type::Speed const current_speed) noexcept -> type::Position {
    type::Position const driver_position{accelerator};

    if (safety_active || !m_is_active.load()) {
      type::Position const inactive_target = process_inactive_cruise(driver_position);
      return common::map_range(inactive_target, control.accelerator_min, control.accelerator_max, control.servo_min, control.servo_max);
    }

    type::Position const cruise_target = process_active_cruise(driver_position, control, current_speed);
    return common::map_range(cruise_target, control.accelerator_min, control.accelerator_max, control.servo_min, control.servo_max);
  }

  auto reset_target() noexcept -> void {
    m_is_active.store(true);
    m_need_reset.store(true);
  }

  auto set_active(bool const active) noexcept -> void { m_is_active.store(active); }

  [[nodiscard]] auto is_active() const noexcept -> bool { return m_is_active.load(); }

  [[nodiscard]] auto get_target_speed() const noexcept -> type::Speed { return m_target_speed; }
};
