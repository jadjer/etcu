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
#include "esp_log.h"
#include "type/control.hpp"
#include "type/type.hpp"

class ControllerCruise {
  static constexpr float critical_task_period_s{0.01f};

  common::AtomicContainer<bool> m_is_enable{false};  // true — скорость установлена и сохранена в памяти
  common::AtomicContainer<bool> m_is_active{false};  // true — ПИД активен и удерживает скорость прямо сейчас
  common::AtomicContainer<bool> m_need_reset{true};

  float m_filtered_speed{0.0f};
  type::Speed m_target_speed{0};
  type::Position m_base_throttle{type::Position::value_min};
  type::Position m_last_position{type::Position::value_min};

  pid_ctrl_block_handle_f_t m_pid_handle{nullptr};

  auto process_inactive_cruise(type::Position const driver_position) noexcept -> type::Position {
    m_is_active.store(false);  // При паузе/тормозе сбрасываем ТОЛЬКО физическую активность в false
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

    bool const is_reset_request = m_need_reset.load();
    if (is_reset_request) {
      m_need_reset.store(false);
    }

    bool const is_active = m_is_active.load();

    if (!is_active || is_reset_request) {
      if (is_reset_request) {
        m_target_speed = current_speed;
        m_base_throttle = driver_position;
      }

      m_is_active.store(true);  // Взводим физическую активность в true
      m_last_position = driver_position;

      pid_reset_ctrl_block(m_pid_handle);
    }

    // Параметры ПИД
    pid_ctrl_parameter_f_t const pid_params = {
        .kp = control.cruise.p,
        .ki = control.cruise.i,
        .kd = control.cruise.d,
        .max_output = type::Position::value_max,
        .min_output = -type::Position::value_max,
        .max_integral = control.cruise.integral_max,
        .min_integral = control.cruise.integral_min,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
    };
    pid_update_parameters(m_pid_handle, &pid_params);

    // ФНЧ скорости ТС
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

    type::Position const cruise_position = m_base_throttle.as<float>() + pid_correction;
    type::Position const cruise_position_minimal{m_last_position.as<float>() - control.cruise.limiter_left.as<float>()};
    type::Position const cruise_position_maximal{m_last_position.as<float>() + control.cruise.limiter_right.as<float>()};
    type::Position const cruise_position_limited = std::clamp(cruise_position.as<float>(), cruise_position_minimal.as<float>(), cruise_position_maximal.as<float>());

    ESP_LOGI("LOG", "SPEED: %f, PID: %f, BASE: %d, POS: %d, LIM: %d", m_filtered_speed, pid_correction, m_base_throttle.get(), cruise_position.get(), cruise_position_limited.get());

    // Перехват управления ручкой газа водителем
    if (driver_position > cruise_position) {
      m_last_position = driver_position;
      return driver_position;
    }

    m_last_position = cruise_position;
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
    pid_new_control_block(&pid_config, &m_pid_handle);
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

    bool const is_active = m_is_active.load();

    // Отсечка по безопасности или если физическая активность сброшена в Паузу (m_is_active == false)
    if (safety_active || !is_active) {
      type::Position const inactive_target = process_inactive_cruise(driver_position);
      return common::map_range(inactive_target, control.accelerator_min, control.accelerator_max, control.servo_min, control.servo_max);
    }

    // Если круиз активен и удерживает скорость (m_is_enable == true && m_is_active == true)
    type::Position const cruise_target = process_active_cruise(driver_position, control, current_speed);
    return common::map_range(cruise_target, control.accelerator_min, control.accelerator_max, control.servo_min, control.servo_max);
  }

  // --- ИНТЕРФЕЙС УПРАВЛЕНИЯ ФЛАГАМИ (Core 0) ---

  /**
   * Принудительный сброс и фиксация новой скорости (LONG Click)
   * Стирает старую цель. Устанавливает enable = true, active = true
   */
  auto reset_target() noexcept -> void {
    m_is_enable.store(true);
    m_is_active.store(true);
    m_need_reset.store(true);
  }

  /**
   * Полное выключение системы (DOUBLE Click)
   * Сбрасывает ВСЕ флаги в false и полностью очищает память
   */
  auto forget_target() noexcept -> void {
    m_is_enable.store(false);
    m_is_active.store(false);
    m_need_reset.store(true);
  }

  auto activate() noexcept -> bool {
    if (bool const is_enabled = m_is_enable.load(); !is_enabled) {
      return false;
    }

    m_is_active.store(true);

    return true;
  }

  auto deactivate() noexcept -> void {
    m_is_active.store(false);
  }

  [[nodiscard]] auto is_enable() const noexcept -> bool { return m_is_enable.load(); }
  [[nodiscard]] auto is_active() const noexcept -> bool { return m_is_active.load(); }
  [[nodiscard]] auto get_target_speed() const noexcept -> type::Speed { return m_target_speed; }
};
