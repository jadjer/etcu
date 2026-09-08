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
#include "common/atomic_container.hpp"
#include "controller_logic.hpp"
#include "device/button.hpp"
#include "logger.hpp"
#include "ota_manager.hpp"
#include "storage.hpp"
#include "system_errors.hpp"
#include "type/calibration.hpp"
#include "type/telemetry.hpp"
#include "type/type.hpp"

template <typename T>
concept AcceleratorConcept = requires(T accelerator,
                                      type::AcceleratorCalibrationData const& calibration_data,
                                      type::Position const& position,
                                      type::AcceleratorTelemetry& accelerator_telemetry) {
  { accelerator.init() } noexcept -> std::same_as<type::SystemError>;
  { accelerator.set_calibration(calibration_data) } noexcept -> std::same_as<void>;
  { accelerator.get_telemetry(accelerator_telemetry) } noexcept -> std::same_as<type::SystemError>;
};

template <typename T>
concept ButtonConcept = requires(T button) {
  { button.init() } noexcept -> std::same_as<type::SystemError>;
  { button.update() } noexcept -> std::same_as<void>;
  { button.has_event() } noexcept -> std::same_as<bool>;
  { button.get_pattern() } noexcept -> std::same_as<std::uint16_t>;
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
  { ecu.get_telemetry(telemetry) } noexcept -> std::same_as<bool>;
};

template <typename T>
concept IndicatorConcept = requires(T indicator) {
  { indicator.init() } noexcept -> std::same_as<type::SystemError>;
  { indicator.update() } noexcept -> std::same_as<bool>;
};

template <typename T>
concept ServoConcept = requires(T servo, type::ServoCalibrationData const& calibration_data, type::Position const& position, type::ServoTelemetry& telemetry) {
  { servo.init() } noexcept -> std::same_as<type::SystemError>;
  { servo.set_calibration(calibration_data) } noexcept -> std::same_as<void>;
  { servo.set_position(position) } noexcept -> std::same_as<type::SystemError>;
  { servo.get_telemetry(telemetry) } noexcept -> std::same_as<type::SystemError>;
};

template <class Accelerator, class Servo, class ECU, class ModeButton, class ModeIndicator, class Brake, class Guard>
  requires AcceleratorConcept<Accelerator> && ServoConcept<Servo> && ECUConcept<ECU> && ButtonConcept<ModeButton> && IndicatorConcept<ModeIndicator> &&
           SwitchConcept<Brake> && SwitchConcept<Guard>
class Controller {
  static constexpr type::AcceleratorCalibrationData accelerator_calibration_factory{
      .hall_a_minimal{650},
      .hall_a_maximal{1350},
      .hall_b_minimal{320},
      .hall_b_maximal{690},
  };
  static constexpr type::ServoCalibrationData servo_calibration_factory{
      .position_minimal{600},
      .position_maximal{1250},
  };

  ECU& m_ecu;
  Servo& m_servo;
  Brake& m_brake;
  Guard& m_guard;
  ModeButton& m_mode_button;
  Accelerator& m_accelerator;
  ModeIndicator& m_mode_indicator;

  common::AtomicContainer<type::Speed> m_target_speed{0};
  common::AtomicContainer<type::Control> m_control{};
  common::AtomicContainer<type::SystemState> m_system_state{type::SystemState::Normal};
  common::AtomicContainer<type::ECUTelemetry> m_ecu_telemetry{};
  common::AtomicContainer<type::DriveTelemetry> m_driver_telemetry{};
  common::AtomicContainer<type::OTAChunk<constants::bluetooth::OTAPayloadSize>> m_ota_chunk{};

  Logger m_logger;
  Storage m_storage;
  ControllerLogic m_logic;
  OTAManager m_ota_manager;
  std::size_t m_chunk_index{std::numeric_limits<std::size_t>::max()};
  SystemErrors m_system_errors;
  type::Control m_last_control{};
  bluetooth::BLEManager m_ble_manager{m_control, m_ota_chunk};

  auto configure() noexcept -> void {
    if (!m_storage.init()) {
      m_logger.log_error("Storage init fault");
    }

    m_storage.load(m_last_control);
    m_control.store(m_last_control);

    type::AcceleratorCalibrationData accelerator_calibration{};

    if (!m_storage.load(accelerator_calibration)) {
      m_logger.log_error("Accelerator factory calibration loading...");

      if (!m_storage.save(accelerator_calibration_factory)) {
        m_logger.log_error("Accelerator factory calibration set error");
      }

      accelerator_calibration = accelerator_calibration_factory;
    }

    m_accelerator.set_calibration(accelerator_calibration);

    type::ServoCalibrationData servo_calibration{};

    if (!m_storage.load(servo_calibration)) {
      m_logger.log_error("Servo factory calibration loading...");

      if (!m_storage.save(servo_calibration_factory)) {
        m_logger.log_error("Servo factory calibration set error");
      }

      servo_calibration = servo_calibration_factory;
    }

    m_servo.set_calibration(servo_calibration);
  }

 public:
  constexpr explicit Controller(Accelerator& accelerator,
                                Servo& servo,
                                ECU& ecu,
                                ModeButton& mode_button,
                                ModeIndicator& mode_indicator,
                                Brake& brake,
                                Guard& guard) noexcept
      : m_ecu(ecu), m_servo(servo), m_brake(brake), m_guard(guard), m_mode_button(mode_button), m_accelerator(accelerator), m_mode_indicator(mode_indicator) {}

  constexpr Controller() noexcept = delete;

  Controller(Controller const&) noexcept = delete;
  auto operator=(Controller const&) noexcept -> Controller& = delete;

  Controller(Controller&&) noexcept = delete;
  auto operator=(Controller&&) noexcept -> Controller& = delete;

  constexpr ~Controller() noexcept = default;

  auto init() noexcept -> void {
    Logger::init();

    m_logger.log_info("Initialization...");

    m_system_errors.update(type::ErrorMaskECU, m_ecu.init());
    m_system_errors.update(type::ErrorMaskServo, m_servo.init());
    m_system_errors.update(type::ErrorMaskPeripheral, m_brake.init());
    m_system_errors.update(type::ErrorMaskGuard, m_guard.init());
    m_system_errors.update(type::ErrorMaskPeripheral, m_mode_button.init());
    m_system_errors.update(type::ErrorMaskAccelerator, m_accelerator.init());
    m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.init());
    m_system_errors.update(type::ErrorMaskIndicator, m_mode_indicator.init());

    m_logger.log_info("Configuration loading...");

    configure();

    m_logger.log_info("Guard checking...");

    if (m_guard.is_active()) {
      m_system_errors.update(type::ErrorMaskGuard, type::SystemError::GuardLock);
      m_system_state.store(type::SystemState::Off);
    }

    if (m_system_errors.has_any()) {
      m_logger.log_error("NOT READY");
    } else {
      m_logger.log_info("READY");
    }
  }

  auto process_ecu_loop() noexcept -> void {
    m_system_errors.update(type::ErrorMaskECU, m_ecu.update());

    if (type::ECUTelemetry ecu_telemetry{}; m_ecu.get_telemetry(ecu_telemetry)) [[unlikely]] {
      m_ecu_telemetry.store(ecu_telemetry);
    }
  }

  auto process_ota_loop() noexcept -> void {
    auto const [size, total, index, data] = m_ota_chunk.load();

    if (size == 0 || index == m_chunk_index) {
      return;
    }

    if (!m_ota_manager.is_active()) {
      if (!m_ota_manager.start_update(size)) {
        m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.send_ota_notify(type::OTAStatus::Error));
        m_chunk_index = std::numeric_limits<std::size_t>::max();
        return;
      }

      m_logger.log_info("Start OTA (Size: %d bytes, Chunks: %d)", size, total);
      m_system_state.store(type::SystemState::Update);
    }

    if (!m_ota_manager.write_chunk(index, data)) {
      m_logger.log_error("Failed to write chunk %d", index);
      m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.send_ota_notify(type::OTAStatus::Error));
      m_system_state.store(type::SystemState::Normal);
      m_chunk_index = std::numeric_limits<std::size_t>::max();
      return;
    }

    m_chunk_index = index;
    m_logger.log_info("Written chunk [%d/%d]", index + 1, total);

    if (index == total - 1) {
      if (!m_ota_manager.end_update()) {
        m_logger.log_error("Failed to finalize OTA update");
        m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.send_ota_notify(type::OTAStatus::Error));
        m_system_state.store(type::SystemState::Normal);
        m_chunk_index = std::numeric_limits<std::size_t>::max();
        return;
      }

      m_logger.log_info("OTA successfully written. Rebooting...");
      m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.send_ota_notify(type::OTAStatus::Completed));

      OTAManager::reboot();

      return;
    }

    m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.send_ota_notify(type::OTAStatus::ReadyForNext));
  }

  auto process_system_loop() noexcept -> void {
     m_mode_button.update();
     m_mode_indicator.update();

     type::Control control = m_control.load();
     type::SystemState const system_state = m_system_state.load();
     type::ECUTelemetry const ecu_telemetry = m_ecu_telemetry.load();

     bool const safety_active = m_brake.is_active() || ecu_telemetry.is_neutral;

     if (control != m_last_control) {
       if (!m_storage.save(control)) [[unlikely]] {
         m_logger.log_error("Control data save error");
       }
     }

     if (system_state == type::SystemState::Normal) {
       if (m_mode_button.has_event()) {
         switch (m_mode_button.get_pattern()) {
           case device::BuildPattern(device::ClickType::Short):
             ESP_LOGI("CTRL", "Short");
             m_target_speed.store(0);
             break;

           case device::BuildPattern(device::ClickType::Long):
             ESP_LOGI("CTRL", "Long");
             if (ecu_telemetry.speed >= control.cruise.threshold && !safety_active) {
               m_target_speed.store(ecu_telemetry.speed);
             }
             break;

           case device::BuildPattern(device::ClickType::Long, device::ClickType::Short):
             ESP_LOGI("CTRL", "Long Short");
             control.servo = type::PositionRange{.min = control.servo.min, .max = 300};
             m_control.store(control);
             break;

           case device::BuildPattern(device::ClickType::Long, device::ClickType::Short, device::ClickType::Short):
             ESP_LOGI("CTRL", "Long Short Short");
             control.servo = type::PositionRange{.min = control.servo.min, .max = 600};
             m_control.store(control);
             break;

           case BuildPattern(device::ClickType::Long, device::ClickType::Short, device::ClickType::Short, device::ClickType::Short):
             ESP_LOGI("CTRL", "Long Short Short Short");
             control.servo = type::PositionRange{.min = control.servo.min, .max = 900};
             m_control.store(control);
             break;

           default:
             break;
         }
       }
     }

    if (safety_active) {
      m_target_speed.store(0);
    }
  }

  auto process_critical_loop() noexcept -> void {
    type::Speed const target_speed = m_target_speed.load();
    type::Control const control = m_control.load();
    type::SystemState const system_state = m_system_state.load();
    type::ECUTelemetry const ecu_telemetry = m_ecu_telemetry.load();

    type::AcceleratorTelemetry accelerator_telemetry{.hall_a = 0, .hall_b = 0, .position = 0};

    if (system_state == type::SystemState::Normal) {
      m_system_errors.update(type::ErrorMaskAccelerator, m_accelerator.get_telemetry(accelerator_telemetry));
    }

    type::Position const servo_position = m_logic.calculate_servo_position(accelerator_telemetry.position, ecu_telemetry.speed, target_speed, control);
    m_system_errors.update(type::ErrorMaskServo, m_servo.set_position(servo_position));

    type::ServoTelemetry servo_telemetry{};
    m_system_errors.update(type::ErrorMaskServo, m_servo.get_telemetry(servo_telemetry));

    type::DriveTelemetry const drive_telemetry{
        .throttle_position = servo_position,
        .servo_telemetry = servo_telemetry,
        .accelerator_telemetry = accelerator_telemetry,
    };
    m_driver_telemetry.store(drive_telemetry);
  }

  auto process_telemetry_loop() noexcept -> void {
    type::Speed const target_speed = m_target_speed.load();
    type::SystemState const system_state = m_system_state.load();
    type::ECUTelemetry const ecu_telemetry = m_ecu_telemetry.load();

    bool const guard_active = m_guard.is_active();
    bool const brake_active = m_brake.is_active();

    auto const [throttle_position, servo_telemetry, accelerator_telemetry] = m_driver_telemetry.load();

    type::SystemTelemetry const system_telemetry{
        .is_guard_active = guard_active,
        .is_brake_enabled = brake_active,

        .ecu_telemetry = ecu_telemetry,
        .servo_telemetry = servo_telemetry,
        .accelerator_telemetry = accelerator_telemetry,

        .target_speed = target_speed,
        .throttle_position = throttle_position,

        .system_state = system_state,
        .system_errors = m_system_errors.get_error_mask(),
    };
    m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.send_telemetry(system_telemetry));
  }
};
