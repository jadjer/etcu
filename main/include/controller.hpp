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

#include "bluetooth/ble_manager.hpp"
#include "commons/atomic_channel.hpp"
#include "commons/atomic_value.hpp"
#include "concepts.hpp"
#include "logic.hpp"
#include "ota_manager.hpp"
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
          concepts::IndicatorConcept StatusInd,
          concepts::IndicatorConcept PowerEnable>
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
  PowerEnable& m_power_enable;

  commons::AtomicChannel<OTAChunk> m_ota_chunk;
  commons::AtomicChannel<BluetoothControl> m_ble_control;
  commons::AtomicChannel<ECUTelemetry> m_ecu_telemetry;
  commons::AtomicChannel<DriveTelemetry> m_driver_telemetry;

  commons::AtomicValue<Speed> m_target_speed{0};
  commons::AtomicValue<Position> m_accelerator_offset{0};
  commons::AtomicValue<SystemState> m_system_state{SystemState::Normal};

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
             StatusInd& status_indicator,
             PowerEnable& power_enable) noexcept
      : m_logger(logger),
        m_accelerator(accelerator),
        m_servo(servo),
        m_ecu(ecu),
        m_mode_button(mode_button),
        m_brake(brake),
        m_guard(guard),
        m_clutch(clutch),
        m_mode_indicator(mode_indicator),
        m_status_indicator(status_indicator),
        m_power_enable(power_enable) {}

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
    m_system_errors.update(m_power_enable.init());

    m_logger.log_info("Load calibration...");

    // TODO Load calibrations from storage

    // if (AcceleratorCalibrationData accelerator_calibration_data; m_storage.load_calibration(accelerator_calibration_data))
    //   m_accelerator.set_calibrate(accelerator_calibration_data);
    //
    // if (ServoCalibrationData servo_calibration_data; m_storage.load_calibration(servo_calibration_data))
    //   m_servo.set_calibrate(servo_calibration_data);

    m_logger.log_info("Check guard...");

    if (m_guard.is_active()) {
      m_system_errors.add(SystemError::GuardLock);
    }

    m_logger.log_info(m_system_errors.has_any() ? "Not ready" : "Ready");

    // TODO Enable servo power
  }

  auto process_system_loop() noexcept -> void {
    // =========================================================================
    // Обновляем информацию по устройствам
    // =========================================================================

    m_mode_button.update();
    m_brake.update();
    m_guard.update();
    m_clutch.update();
    m_system_errors.update(m_ecu.update());
    m_system_errors.update(m_mode_indicator.update());
    m_system_errors.update(m_status_indicator.update());

    // =========================================================================
    // Собираем данные по устройствам
    // =========================================================================

    bool const guard_active = m_guard.is_active();
    bool const brake_active = m_brake.is_active();
    bool const clutch_active = m_clutch.is_active();
    Speed const target_speed = m_target_speed.get();
    Position const accelerator_offset = m_accelerator_offset.get();
    SystemState const system_state = m_system_state.get();

    Position throttle_position{0};
    Position accelerator_position{0};
    ECUTelemetry ecu_telemetry{};
    ServoTelemetry servo_telemetry{};

    // =========================================================================
    // Обновляем телеметрию по ECU и отправляем на исполнительное ядро
    // =========================================================================

    {
      m_system_errors.update(m_ecu.get_telemetry(ecu_telemetry));
      std::ignore = m_ecu_telemetry.send(ecu_telemetry);
    }

    // =========================================================================
    // Проверяем контроль через BLE
    // =========================================================================

    if (BluetoothControl control; m_ble_control.receive(control)) {
      if (control.sync_enabled) {
        m_logger.log_info("Start calibrate...");
        m_system_state.set(SystemState::Calibration);
      }

      m_accelerator_offset.set(control.accelerator_offset);
    }

    // =========================================================================
    // Отображаем ошибки
    // =========================================================================

    if (m_system_errors.has_any()) [[unlikely]] {
      m_logger.log_active_errors(m_system_errors.get_all());
    }

    // =========================================================================
    // Обрабатываем в соответствии с режимом
    // =========================================================================

    switch (system_state) {
      case SystemState::Off:
        return;

      case SystemState::Normal: {
        if (m_mode_button.is_long_press()) {
          m_target_speed.set(ecu_telemetry.speed);
          m_logger.log_info("Set offset as %d", accelerator_position);
        }
      } break;

      case SystemState::Calibration: {
        AcceleratorCalibrationData accelerator_calibration_data;
        m_system_errors.update(m_accelerator.calibrate(accelerator_calibration_data));

        if (m_mode_button.is_short_press()) {
          m_logger.log_info("Servo calibrate ...");

          ServoCalibrationData servo_calibration_data;
          m_servo.calibrate(servo_calibration_data);

          m_logger.log_info("Save calibration...");

          std::ignore = m_storage.save_calibration(accelerator_calibration_data);
          std::ignore = m_storage.save_calibration(servo_calibration_data);

          m_system_state.set(SystemState::Normal);

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

    // =========================================================================
    // Отправка телеметрии через BLE
    // =========================================================================

    if (DriveTelemetry telemetry; m_driver_telemetry.receive(telemetry)) {
      accelerator_position = telemetry.accelerator_position;
      throttle_position = telemetry.throttle_position;
      servo_telemetry = telemetry.servo_telemetry;
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
  }

  auto process_critical_loop() noexcept -> void {
    SystemState const system_state = m_system_state.get();

    Speed target_speed{0};
    Speed current_speed{0};
    Position accelerator_offset{0};
    Position accelerator_position{0};
    ServoTelemetry servo_telemetry{};

    if (system_state == SystemState::Normal) {
      if (ECUTelemetry ecu_telemetry; m_ecu_telemetry.receive(ecu_telemetry)) {
        current_speed = ecu_telemetry.speed;
      }

      target_speed = m_target_speed.get();
      accelerator_offset = m_accelerator_offset.get();

      m_system_errors.update(m_accelerator.get_position(accelerator_position));
    }

    Position const throttle_position = m_logic.calculate_servo_position(accelerator_offset, accelerator_position, current_speed, target_speed);

    m_servo.set_position(throttle_position);
    m_servo.get_telemetry(servo_telemetry);

    DriveTelemetry const drive_telemetry{
        .servo_telemetry = servo_telemetry,
        .accelerator_position = accelerator_position,
        .throttle_position = throttle_position,
    };
    std::ignore = m_driver_telemetry.send(drive_telemetry);
  }
};
