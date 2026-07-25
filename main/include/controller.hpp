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
#include "logger.hpp"
#include "logic.hpp"
#include "storage.hpp"
#include "system_errors.hpp"
#include "types.hpp"

template <concepts::LoggerConcept Logger,
          concepts::AcceleratorConcept Accelerator,
          concepts::ServoConcept Servo,
          concepts::ECUConcept ECU,
          concepts::ButtonConcept ModeBtn,
          concepts::ButtonConcept Brake,
          concepts::ButtonConcept Guard,
          concepts::ButtonConcept Clutch,
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

  Logic m_logic;
  Storage m_storage;
  SystemErrors m_system_errors;

  SharedData m_shared_data;
  std::atomic<Mode> m_current_mode{Mode::Off};
  std::atomic<Speed> m_target_speed{0};
  std::atomic<bool> m_shared_data_ready{false};
  std::atomic<bool> m_offset_accelerator{false};

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
    m_logic.init();
    m_logger.init();

    m_logger.log_info("Initialization...");

    m_system_errors.update(m_accelerator.init());
    m_system_errors.update(m_servo.init());
    m_system_errors.update(m_ecu.init());
    m_system_errors.update(m_mode_button.init());
    m_system_errors.update(m_brake.init());
    m_system_errors.update(m_guard.init());
    m_system_errors.update(m_clutch.init());
    m_system_errors.update(m_mode_indicator.init());
    m_system_errors.update(m_status_indicator.init());

    m_logger.log_info("Load storage data...");

    CalibrationData const calibration_data{
        .hall_a_minimal = 12,
        .hall_a_maximal = 555,
        .hall_b_minimal = 45,
        .hall_b_maximal = 123,
    };
    std::ignore = m_storage.save_calibration(calibration_data);

    m_logger.log_info("Check guard...");

    if (m_guard.is_active()) {
      m_current_mode.store(Mode::Off, std::memory_order_relaxed);
      m_system_errors.add(SystemError::GuardLock);
      m_logger.log_info("Guard enabled. Servo disabled");

      return;
    }

    m_logger.log_info("Test servo...");

    m_system_errors.update(m_servo.self_test());

    // if () {
    //   m_current_mode.store(Mode::Off, std::memory_order_relaxed);
    //   m_system_errors.add(SystemError::ServoMechanicalFault);
    //   m_logger.log_info("Self test failed. Servo disabled");
    //
    //   return;
    // }

    m_current_mode.store(Mode::Normal, std::memory_order_relaxed);

    m_logger.log_info("Ready");
  }

  auto process_system_loop() noexcept -> void {
    Mode const current_mode = m_current_mode.load(std::memory_order_relaxed);
    SystemError current_errors = m_system_errors.get_all();

    m_ecu.update();
    m_mode_button.update();
    m_brake.update();
    m_guard.update();
    m_clutch.update();
    m_mode_indicator.update();
    m_status_indicator.update();

    if (current_mode == Mode::Normal) {
      if (m_mode_button.is_long_then(500)) {
        m_current_mode.store(Mode::Calibration, std::memory_order_relaxed);
        m_logger.log_info("Start calibration...");
      }
    }

    if (current_mode == Mode::Calibration) {
      if (m_mode_button.is_short_press()) {
        m_current_mode.store(Mode::Normal, std::memory_order_relaxed);
        m_logger.log_info("Calibration finished. Returning to Normal.");
      }
    }

    if (current_errors != SystemError::None && current_mode != Mode::Off) [[unlikely]] {
      m_current_mode.store(Mode::Off, std::memory_order_relaxed);
      m_logger.log_active_errors(current_errors);
    }

    m_logger.log_active_errors(current_errors);

    m_system_errors.update(current_errors);

    // m_mode_indicator.set_status(m_current_mode.load(std::memory_order_relaxed), current_errors);
  }

  auto process_critical_loop() noexcept -> void {
    static SharedData shared_data{};
    static ServoTelemetry servo_telemetry{};

    Mode const local_mode = m_current_mode.load(std::memory_order_relaxed);
    SystemError local_errors = m_system_errors.get_all();

    if (local_mode == Mode::Calibration) {
      std::ignore = m_servo.set_position(0);

      SystemError const err = m_accelerator.calibrate();

      m_system_errors.update(err);

    } else {
      // if (m_shared_data_ready.load(std::memory_order_acquire)) {
      //   shared_data = m_shared_data;
      //   m_shared_data_ready.store(false, std::memory_order_relaxed);
      // }

      Position accelerator_position = 0;

      SystemError const err = m_accelerator.get_position(accelerator_position);

      m_system_errors.update(err);

      m_logger.log_info("Position %d", accelerator_position);

      // m_servo.read_telemetry(servo_telemetry, local_errors);
      //
      // Position const computed_position = m_logic.calculate_servo_position(local_mode, accelerator_position, false, 0, 0);
      //
      // m_servo.set_position(computed_position, local_errors);
    }

    m_system_errors.update(local_errors);
  }
};
