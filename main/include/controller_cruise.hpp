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
  static constexpr float filter_alpha{0.10f};

  float m_filtered_speed{0};

  common::AtomicContainer<bool> m_is_enable{false};
  common::AtomicContainer<bool> m_is_active{false};
  common::AtomicContainer<bool> m_need_reset{true};

  common::AtomicContainer<float> m_error{0.0f};
  common::AtomicContainer<float> m_correction{0.0f};

  common::AtomicContainer<type::Speed> m_target_speed{0};
  common::AtomicContainer<type::Position> m_target_position{0};

  pid_ctrl_block_handle_f_t m_pid_handle{nullptr};

  auto get_smooth_speed(float const current_speed) noexcept -> float {
    if (m_filtered_speed <= 0) {
      m_filtered_speed = current_speed;
    }

    m_filtered_speed = m_filtered_speed * (1.0f - filter_alpha) + current_speed * filter_alpha;

    return m_filtered_speed;
  }

  auto process_inactive_cruise(type::Position const driver_position) noexcept -> type::Position {
    m_is_active.store(false);

    if (m_pid_handle != nullptr) {
      pid_reset_ctrl_block_f(m_pid_handle);
    }

    return driver_position;
  }

  auto process_active_cruise(type::Position const driver_position,
                             type::Speed const current_speed,
                             type::RPM const current_rpm,
                             type::Cruise const& control) noexcept -> type::Position {
    if (m_pid_handle == nullptr) [[unlikely]] {
      return process_inactive_cruise(driver_position);
    }

    if (bool const is_reset_request = m_need_reset.load(); is_reset_request) {
      m_need_reset.store(false);
      m_target_speed.store(current_speed);
      m_target_position.store(driver_position);
      pid_reset_ctrl_block_f(m_pid_handle);
    }

    type::Speed const target_speed = m_target_speed.load();
    auto const target_speed_f = target_speed.as<float>();
    auto const current_speed_f = current_speed.as<float>();
    float const smooth_speed_f = get_smooth_speed(current_speed_f);
    float const error_f = target_speed_f - smooth_speed_f;
    m_error.store(error_f);

    pid_ctrl_parameter_f_t pid_params{
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .max_output = control.limiter_up.as<float>(),
        .min_output = -control.limiter_down.as<float>(),
        .max_integral = 0.0f,
        .min_integral = 0.0f,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
    };

    if (error_f >= 0.0f) {
      pid_params.kp = control.acc.p;
      pid_params.ki = control.acc.i;
      pid_params.kd = control.acc.d;
    } else {
      pid_params.kp = control.dec.p;
      pid_params.ki = control.dec.i;
      pid_params.kd = control.dec.d;
    }

    if (pid_update_parameters_f(m_pid_handle, &pid_params) != ESP_OK) [[unlikely]] {
      return process_inactive_cruise(driver_position);
    }

    float pid_correction_f{0.0f};
    if (pid_compute_f(m_pid_handle, error_f, &pid_correction_f) != ESP_OK) [[unlikely]] {
      return process_inactive_cruise(driver_position);
    }
    m_correction.store(pid_correction_f);

    type::Position const target_position = m_target_position.load();
    type::Position const cruise_position = target_position + pid_correction_f;

    if (driver_position > cruise_position) {
      pid_reset_ctrl_block_f(m_pid_handle);
      return driver_position;
    }

    return cruise_position;
  }

 public:
  constexpr ControllerCruise() noexcept = default;

  ControllerCruise(ControllerCruise const&) noexcept = delete;
  auto operator=(ControllerCruise const&) noexcept -> ControllerCruise& = delete;

  ControllerCruise(ControllerCruise&&) noexcept = delete;
  auto operator=(ControllerCruise&&) noexcept -> ControllerCruise& = delete;

  constexpr ~ControllerCruise() noexcept = default;

  auto init() noexcept -> bool {
    static constexpr pid_ctrl_config_f_t pid_config{
        .init_param =
            {
                .kp = 0.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .max_output = 0.0f,
                .min_output = 0.0f,
                .max_integral = 0.0f,
                .min_integral = 0.0f,
                .cal_type = PID_CAL_TYPE_INCREMENTAL,
            },
    };
    if (pid_new_control_block_f(&pid_config, &m_pid_handle) != ESP_OK) [[unlikely]] {
      return false;
    }

    return true;
  }

  [[nodiscard]] auto generate_throttle_position(type::Position const accelerator,
                                                type::Speed const current_speed,
                                                type::RPM const current_rpm,
                                                bool const is_safety_active,
                                                type::Control const& control) noexcept -> type::Position {
    type::Position target_position{accelerator};

    bool const is_enabled = m_is_enable.load();  // NOLINT
    bool const is_active = m_is_active.load();   // NOLINT

    if (!is_enabled || !is_active || is_safety_active) {
      target_position = process_inactive_cruise(accelerator);
    } else {
      target_position = process_active_cruise(accelerator, current_speed, current_rpm, control.cruise);
    }

    return common::map_range(target_position, control.accelerator.min, control.accelerator.max, control.servo.min, control.servo.max);
  }

  [[nodiscard]] auto is_enable() const noexcept -> bool { return m_is_enable.load(); }
  [[nodiscard]] auto is_active() const noexcept -> bool { return m_is_active.load(); }
  [[nodiscard]] auto get_error() const noexcept -> float { return m_error.load(); }
  [[nodiscard]] auto get_correction() const noexcept -> float { return m_correction.load(); }
  [[nodiscard]] auto get_target_speed() const noexcept -> type::Speed { return m_target_speed.load(); }
  [[nodiscard]] auto get_target_position() const noexcept -> type::Position { return m_target_position.load(); }

  auto enable() noexcept -> void {
    m_is_enable.store(true);
    m_is_active.store(true);
    m_need_reset.store(true);
  }

  auto disable() noexcept -> void {
    m_is_enable.store(false);
    m_is_active.store(false);
    m_need_reset.store(true);
  }

  auto resume() noexcept -> bool {
    if (bool const is_enabled = m_is_enable.load(); !is_enabled) {
      return false;
    }

    m_is_active.store(true);
    return true;
  }

  auto pause() noexcept -> bool {
    if (bool const is_enabled = m_is_enable.load(); !is_enabled) {
      return false;
    }

    m_is_active.store(false);
    return true;
  }
};
