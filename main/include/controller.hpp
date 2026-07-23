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

#include <atomic>
#include "concepts/concepts.hpp"
#include "core.hpp"
#include "types.hpp"

template <concepts::LoggerConcept Logger,
          concepts::AcceleratorConcept Accelerator,
          concepts::ServoConcept Servo,
          concepts::ECUConcept ECU,
          concepts::ButtonConcept ModeBtn,
          concepts::SwitchConcept Brake,
          concepts::SwitchConcept Guard,
          concepts::SwitchConcept Clutch,
          concepts::IndicatorConcept ModeInd,
          concepts::IndicatorConcept StatusInd>
class Controller {
  Logger& m_logger;
  Accelerator& m_accelerator;
  Servo& m_servo;
  ECU& m_ecu;
  ModeBtn& m_mode_button;
  Brake& m_brake;
  Guard& m_guard;
  Clutch& m_clutch;
  ModeInd& m_mode_indicator;
  StatusInd& m_status_indicator;

  Core m_core;
  SharedData m_shared_data;

  std::atomic<Mode> m_current_mode{Mode::Normal};
  std::atomic<std::uint32_t> m_system_errors{static_cast<std::uint32_t>(SystemError::None)};
  std::atomic<bool> m_shared_data_ready{false};
  std::atomic<std::uint16_t> m_target_speed{0};

 public:
  Controller(Logger& logger,
             Accelerator& accelerator,
             Servo& servo,
             ECU& ecu,
             ModeBtn& mode_button,
             Brake& brake,
             Guard& guard,
             Clutch& clutch,
             ModeInd& mode_indicator,
             StatusInd& status_indicator) noexcept
      : m_logger(logger),
        m_accelerator(accelerator),
        m_servo(servo),
        m_ecu(ecu),
        m_mode_button(mode_button),
        m_brake(brake),
        m_guard(guard),
        m_clutch(clutch),
        m_mode_indicator(mode_indicator),
        m_status_indicator(status_indicator) {}

  auto init() noexcept -> void {
    m_logger.init();
    m_logger.log_info("Starting Throttle Controller Initialization...");

    m_core.init();

    m_accelerator.init();
    m_servo.init();
    m_ecu.init();
    m_mode_button.init();
    m_brake.init();
    m_guard.init();
    m_clutch.init();
    m_mode_indicator.init();
    m_status_indicator.init();

    m_logger.log_info("Executing automatic zero and range calibration...");
    // m_core.execute_auto_calibration();
  }

  auto process_system_loop() noexcept -> void {
    m_ecu.update();
    m_mode_button.update();

    std::uint16_t const rpm = m_ecu.get_rpm();
    std::uint16_t const tps = m_ecu.get_tps();
    std::uint16_t const speed = m_ecu.get_speed();
    auto const current_errors = static_cast<SystemError>(m_system_errors.load(std::memory_order_relaxed));

    Mode current_mode = m_current_mode.load(std::memory_order_relaxed);

    if (m_guard.is_active()) {
      current_mode = Mode::Off;
      m_current_mode.store(current_mode, std::memory_order_relaxed);
      m_system_errors.fetch_or(static_cast<std::uint32_t>(SystemError::GuardLock), std::memory_order_relaxed);
    }

    if (current_errors != SystemError::None) {
      m_current_mode.store(Mode::Off, std::memory_order_relaxed);
    }

    if (m_clutch.is_active() || m_brake.is_active()) {
      m_target_speed.store(0, std::memory_order_relaxed);
    }

    if (m_mode_button.is_long_press() && speed == 0) {
      m_logger.log_info("Offset zero position");
    }

    if (m_mode_button.is_long_press() && speed > 60) {
      m_target_speed.store(speed, std::memory_order_relaxed);
      m_logger.log_info("Enable Cruise Control");
    }

    m_mode_indicator.set_status(current_mode, current_errors);
  }

  auto process_critical_loop() noexcept -> void {
    static SharedData shared_data{};
    static ServoTelemetry servo_telemetry{};

    auto local_errors = static_cast<SystemError>(m_system_errors.load(std::memory_order_relaxed));
    Mode const local_mode = m_current_mode.load(std::memory_order_relaxed);

    if (m_shared_data_ready.load(std::memory_order_acquire)) {
      shared_data = m_shared_data;
      m_shared_data_ready.store(false, std::memory_order_relaxed);
    }

    std::uint16_t const accelerator_position = m_accelerator.get_position(local_errors);

    m_servo.read_telemetry(servo_telemetry, local_errors);

    std::uint16_t const computed_position = m_core.calculate_servo_position(local_mode, accelerator_position, false, 0, 0);

    m_servo.set_position(computed_position, local_errors);

    m_system_errors.store(static_cast<std::uint32_t>(local_errors), std::memory_order_release);
  }
};
