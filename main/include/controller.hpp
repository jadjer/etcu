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
#include "common/atomic_channel.hpp"
#include "common/atomic_value.hpp"
#include "common/storage.hpp"
#include "config/concepts.hpp"
#include "controller_logic.hpp"
#include "logger.hpp"
#include "ota_manager.hpp"
#include "system_errors.hpp"
#include "type/calibration.hpp"
#include "type/telemetry.hpp"
#include "type/type.hpp"

template <typename T>
concept AcceleratorConcept =
    requires(T accelerator, type::AcceleratorCalibrationData calibration_data, type::Position const& position, type::Position& position_result) {
      { accelerator.init() } noexcept -> std::same_as<type::SystemError>;
      { accelerator.set_calibration(calibration_data) } noexcept -> std::same_as<void>;
      { accelerator.get_position(position_result) } noexcept -> std::same_as<type::SystemError>;
    };

template <typename T>
concept ButtonConcept = requires(T button) {
  { button.init() } noexcept -> std::same_as<type::SystemError>;
  { button.update() } noexcept -> std::same_as<void>;
  { button.is_active() } noexcept -> std::same_as<bool>;
  { button.is_short_press() } noexcept -> std::same_as<bool>;
  { button.is_long_press() } noexcept -> std::same_as<bool>;
};

template <typename T>
concept SwitchConcept = requires(T s) {
  { s.init() } noexcept -> std::same_as<type::SystemError>;
  { s.is_active() } noexcept -> std::same_as<bool>;
};

template <typename T>
concept ECUConcept = requires(T ecu, type::ECUTelemetry telemetry) {
  { ecu.init() } noexcept -> std::same_as<type::SystemError>;
  { ecu.update() } noexcept -> std::same_as<type::SystemError>;
  { ecu.get_telemetry(telemetry) } noexcept -> std::same_as<type::SystemError>;
};

template <typename T>
concept IndicatorConcept = requires(T indicator) {
  { indicator.init() } noexcept -> std::same_as<type::SystemError>;
  { indicator.update() } noexcept -> std::same_as<type::SystemError>;
};

template <typename T>
concept ServoConcept = requires(T servo, type::Position const& position, type::ServoTelemetry& telemetry) {
  { servo.init() } noexcept -> std::same_as<type::SystemError>;
  { servo.set_position(position) } noexcept -> std::same_as<void>;
  { servo.get_telemetry(telemetry) } noexcept -> std::same_as<bool>;
};

struct DriveTelemetry {
  type::ServoTelemetry servo_telemetry{};
  type::Position accelerator_position{};
  type::Position throttle_position{};
};

template <class Accelerator, class Servo, class ECU, class ModeButton, class ModeIndicator, class Brake, class Guard>
  requires AcceleratorConcept<Accelerator> && ServoConcept<Servo> && ECUConcept<ECU> && ButtonConcept<ModeButton> && IndicatorConcept<ModeIndicator> &&
           SwitchConcept<Brake> && SwitchConcept<Guard>

class Controller {
  ECU& m_ecu;
  Servo& m_servo;
  Brake& m_brake;
  Guard& m_guard;
  ModeButton& m_mode_button;
  Accelerator& m_accelerator;
  ModeIndicator& m_mode_indicator;

  common::AtomicChannel<type::OTAChunk<constants::bluetooth::MaxBlePayloadSize>> m_ota_chunk;
  common::AtomicChannel<type::ECUTelemetry> m_ecu_telemetry;
  common::AtomicChannel<type::BluetoothControl> m_ble_control;
  common::AtomicChannel<DriveTelemetry> m_driver_telemetry;

  common::AtomicValue<type::Speed> m_target_speed{type::Speed{0}};
  common::AtomicValue<type::Position> m_accelerator_offset{type::Position{0}};
  common::AtomicValue<type::SystemState> m_system_state{type::SystemState::Normal};

  Logic m_logic;
  Logger m_logger;
  common::Storage m_storage;
  SystemErrors m_system_errors;
  update::OTAManager m_ota_manager;
  bluetooth::BLEManager m_ble_manager{m_ota_chunk, m_ble_control};

 public:
  constexpr explicit Controller(Accelerator& accelerator,
                                Servo& servo,
                                ECU& ecu,
                                ModeButton& mode_button,
                                ModeIndicator& mode_indicator,
                                Brake& brake,
                                Guard& guard) noexcept
      : m_ecu(ecu), m_servo(servo), m_brake(brake), m_guard(guard), m_mode_button(mode_button), m_accelerator(accelerator), m_mode_indicator(mode_indicator) {}

  auto init() noexcept -> void {
    m_logger.init();

    m_logger.log_info("Initialization...");

    m_system_errors.update(m_ecu.init());
    m_system_errors.update(m_servo.init());
    m_system_errors.update(m_brake.init());
    m_system_errors.update(m_guard.init());
    m_system_errors.update(m_mode_button.init());
    m_system_errors.update(m_accelerator.init());
    m_system_errors.update(m_ble_manager.init());
    m_system_errors.update(m_mode_indicator.init());

    m_logger.log_info("Load calibration...");

    // TODO Load calibrations from storage

    // if (AcceleratorCalibrationData accelerator_calibration_data; m_storage.load_calibration(accelerator_calibration_data))
    //   m_accelerator.set_calibrate(accelerator_calibration_data);
    //
    // if (ServoCalibrationData servo_calibration_data; m_storage.load_calibration(servo_calibration_data))
    //   m_servo.set_calibrate(servo_calibration_data);

    m_logger.log_info("Check guard...");

    if (m_guard.is_active()) {
      m_system_errors.add(type::SystemError::GuardLock);
    }

    m_logger.log_info("{}", m_system_errors.has_any() ? "Not ready" : "Ready");

    // TODO Enable servo power
  }

  auto process_system_loop() noexcept -> void {
    // =========================================================================
    // Обновляем информацию по устройствам
    // =========================================================================

    m_mode_button.update();
    m_system_errors.update(m_ecu.update());
    m_system_errors.update(m_mode_indicator.update());

    // =========================================================================
    // Собираем данные по устройствам
    // =========================================================================

    bool const guard_active = m_guard.is_active();
    bool const brake_active = m_brake.is_active();
    type::Speed const target_speed = m_target_speed.get();
    type::Position const accelerator_offset = m_accelerator_offset.get();
    type::SystemState const system_state = m_system_state.get();

    type::Position throttle_position{0};
    type::Position accelerator_position{0};
    type::ECUTelemetry ecu_telemetry{};
    type::ServoTelemetry servo_telemetry{};

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

    if (type::BluetoothControl control; m_ble_control.receive(control)) {
      if (control.sync_enabled) {
        m_logger.log_info("Start calibrate...");
        m_system_state.set(type::SystemState::Calibration);
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
      case type::SystemState::Off:
        return;

      case type::SystemState::Normal: {
        if (m_mode_button.is_long_press()) {
          m_target_speed.set(ecu_telemetry.speed);
          m_logger.log_info("Set offset as %d", accelerator_position.value);
        }
      } break;

      case type::SystemState::Calibration: {
        type::AcceleratorCalibrationData accelerator_calibration_data;
        // m_system_errors.update(m_accelerator.calibrate(accelerator_calibration_data));

        if (m_mode_button.is_short_press()) {
          m_logger.log_info("Servo calibrate ...");

          // type::ServoCalibrationData servo_calibration_data;
          // m_servo.calibrate(servo_calibration_data);

          m_logger.log_info("Save calibration...");

          std::ignore = m_storage.save_calibration(accelerator_calibration_data);
          // std::ignore = m_storage.save_calibration(servo_calibration_data);

          m_system_state.set(type::SystemState::Normal);

          m_logger.log_info("Calibration finished. Returning to Normal.");
        }
      } break;

      case type::SystemState::Update: {
        type::OTAChunk<constants::bluetooth::MaxBlePayloadSize> chunk;

        if (bool const is_received = m_ota_chunk.receive(chunk)) {
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

    type::SystemTelemetry const system_telemetry{
        .is_guard_active = guard_active,
        .is_brake_enabled = brake_active,

        .servo_telemetry = servo_telemetry,
        .ecu_telemetry = ecu_telemetry,
        .accelerator_position = accelerator_position,
        .accelerator_offset = accelerator_offset,
        .throttle_position = throttle_position,
        .target_speed = target_speed,

        .system_state = system_state,
        .system_errors = m_system_errors.get_all(),
    };
    m_system_errors.update(m_ble_manager.send_telemetry(system_telemetry));
  }

  auto process_critical_loop() noexcept -> void {
    type::SystemState const system_state = m_system_state.get();

    type::Speed target_speed{0};
    type::Speed current_speed{0};
    type::Position accelerator_offset{0};
    type::Position accelerator_position{0};
    type::ServoTelemetry servo_telemetry{};

    if (system_state == type::SystemState::Normal) {
      if (type::ECUTelemetry ecu_telemetry; m_ecu_telemetry.receive(ecu_telemetry)) {
        current_speed = ecu_telemetry.speed;
      }

      target_speed = m_target_speed.get();
      accelerator_offset = m_accelerator_offset.get();

      m_system_errors.update(m_accelerator.get_position(accelerator_position));
    }

    type::Position const throttle_position = m_logic.calculate_servo_position(accelerator_position, accelerator_offset, current_speed, target_speed);

    m_servo.set_position(throttle_position);
    std::ignore = m_servo.get_telemetry(servo_telemetry);

    DriveTelemetry const drive_telemetry{
        .servo_telemetry = servo_telemetry,
        .accelerator_position = accelerator_position,
        .throttle_position = throttle_position,
    };
    std::ignore = m_driver_telemetry.send(drive_telemetry);
  }
};
