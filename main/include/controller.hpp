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

#include "bluetooth/ble_manager.hpp"
#include "commons/atomic_channel.hpp"
#include "concepts/concepts.hpp"
#include "logger.hpp"
#include "logic.hpp"
#include "storage.hpp"
#include "system_errors.hpp"
#include "types.hpp"
#include "update/ota_manager.hpp"

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

  commons::AtomicChannel<OTAChunk> m_ota_chunk;
  commons::AtomicChannel<BluetoothControl> m_ble_control;
  commons::AtomicChannel<ECUTelemetry> m_ecu_telemetry;
  commons::AtomicChannel<DriveTelemetry> m_driver_telemetry;

  std::atomic<Speed> m_target_speed{0};
  std::atomic<Position> m_accelerator_offset{0};
  std::atomic<SystemState> m_system_state{SystemState::Normal};

  Logic m_logic;
  Storage m_storage;
  SystemErrors m_system_errors;
  update::OTAManager m_ota_manager;
  bluetooth::BLEManager m_ble_manager{m_ota_chunk, m_ble_control};

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
    m_system_errors.update(m_ble_manager.init());

    m_logger.log_info("Load calibration...");

    CalibrationData calibration_data{};
    std::ignore = m_storage.load_calibration(calibration_data);

    m_logger.log_info("Check guard...");

    if (m_guard.is_active()) {
      m_system_errors.add(SystemError::GuardLock);
    }

    m_logger.log_info("Test servo...");

    m_system_errors.update(m_servo.self_test());

    m_logger.log_info(m_system_errors.has_any() ? "Not ready" : "Ready");
  }

  auto process_system_loop() noexcept -> void {
    m_mode_button.update();
    m_brake.update();
    m_guard.update();
    m_clutch.update();
    m_system_errors.update(m_ecu.update());
    m_system_errors.update(m_mode_indicator.update());
    m_system_errors.update(m_status_indicator.update());

    Speed const target_speed = m_target_speed.load(std::memory_order_relaxed);
    Position const accelerator_offset = m_accelerator_offset.load(std::memory_order_relaxed);

    Position throttle_position{0};
    Position accelerator_position{0};
    ECUTelemetry ecu_telemetry{};
    ServoTelemetry servo_telemetry{};
    bool const guard_active = m_guard.is_active();
    bool const brake_active = m_brake.is_active();
    bool const clutch_active = m_clutch.is_active();
    SystemState const system_state = m_system_state.load(std::memory_order_relaxed);

    {
      m_system_errors.update(m_ecu.get_telemetry(ecu_telemetry));
      std::ignore = m_ecu_telemetry.send(ecu_telemetry);
    }
    {
      DriveTelemetry telemetry;
      bool const is_received = m_driver_telemetry.receive(telemetry);
      if (is_received) {
        accelerator_position = telemetry.accelerator_position;
        throttle_position = telemetry.throttle_position;
        servo_telemetry = telemetry.servo_telemetry;
      }
    }

    SystemTelemetry const system_telemetry{
        .servo_telemetry = servo_telemetry,
        .ecu_telemetry = ecu_telemetry,
        .accelerator_position = accelerator_position,
        .accelerator_offset = accelerator_offset,
        .throttle_position = throttle_position,
        .target_speed = target_speed,
        .guard_active = guard_active,
        .brake_enabled = brake_active,
        .clutch_enabled = clutch_active,
        .system_state = system_state,
        .system_errors = m_system_errors.get_all(),
    };
    m_system_errors.update(m_ble_manager.send_telemetry(system_telemetry));

    if (BluetoothControl control; m_ble_control.receive(control)) {
      m_logger.log_info("BLE control. Reset %d, Sync %d", control.error_reset, control.sync_enabled);

      if (control.error_reset) {
        m_system_errors.reset();
      }

      if (control.sync_enabled) {
        m_system_state.store(SystemState::Calibration, std::memory_order_relaxed);
      }
    }

    if (m_system_errors.has_any()) [[unlikely]] {
      // m_system_state.store(SystemState::Off, std::memory_order_relaxed);
      // m_logger.log_info("Has errors. Servo disabled");
      m_logger.log_active_errors(m_system_errors.get_all());
    }

    switch (system_state) {
      case SystemState::Off:
        return;
      case SystemState::Normal:
        if (m_mode_button.is_long_press()) {
          m_accelerator_offset.store(accelerator_position, std::memory_order_relaxed);
          m_logger.log_info("Set offset as %d", accelerator_position);
        }
        break;
      case SystemState::Calibration: {
        m_system_errors.update(m_accelerator.calibrate());

        if (m_mode_button.is_long_press()) {
          m_system_state.store(SystemState::Normal, std::memory_order_relaxed);
          m_logger.log_info("Calibration finished. Returning to Normal.");
        }
      } break;

      case SystemState::Update: {
        OTAChunk chunk;

        bool const is_received = m_ota_chunk.receive(chunk);
        if (is_received) {
          // if (!m_ota_manager.isActive())
          //   m_ota_manager.startUpdate();
          //
          // if (m_ota_manager.writeChunk(chunk.chunk)) {
          //   m_system_errors.update(m_ble_manager.send_ota_notify(OTAStatus::ReadyForNext));
          // } else {
          //   m_system_errors.update(m_ble_manager.send_ota_notify(OTAStatus::ErrorOccurred));
          // }
        }
      } break;
    }
  }

  auto process_critical_loop() noexcept -> void {
    SystemState const system_state = m_system_state.load(std::memory_order_relaxed);

    Speed target_speed{0};
    Speed current_speed{0};
    Position accelerator_offset{0};
    Position accelerator_position{0};
    ServoTelemetry servo_telemetry{};

    if (system_state == SystemState::Normal) {
      if (ECUTelemetry ecu_telemetry; m_ecu_telemetry.receive(ecu_telemetry)) {
        current_speed = ecu_telemetry.speed;
      }

      target_speed = m_target_speed.load(std::memory_order_relaxed);
      accelerator_offset = m_accelerator_offset.load(std::memory_order_relaxed);

      m_system_errors.update(m_accelerator.get_position(accelerator_position));
    }

    Position const throttle_position = m_logic.calculate_servo_position(accelerator_offset, accelerator_position, current_speed, target_speed);

    m_system_errors.update(m_servo.set_position(throttle_position));
    m_system_errors.update(m_servo.get_telemetry(servo_telemetry));

    DriveTelemetry const drive_telemetry{
        .servo_telemetry = servo_telemetry,
        .accelerator_position = accelerator_position,
        .throttle_position = throttle_position,
    };
    std::ignore = m_driver_telemetry.send(drive_telemetry);
  }
};
