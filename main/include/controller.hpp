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
#include "common/storage.hpp"
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

  common::AtomicContainer<type::Speed> m_target_speed{0};
  common::AtomicContainer<type::SystemState> m_system_state{type::SystemState::Normal};

  common::AtomicContainer<type::Control> m_control{};
  common::AtomicContainer<type::ECUTelemetry> m_ecu_telemetry{};
  common::AtomicContainer<type::DriveTelemetry> m_driver_telemetry{};
  common::AtomicContainer<type::OTAChunk<constants::bluetooth::OTAPayloadSize>> m_ota_chunk{};

  Logic m_logic;
  Logger m_logger;
  OTAManager m_ota_manager;
  SystemErrors m_system_errors;
  bluetooth::BLEManager m_ble_manager{m_control, m_ota_chunk};

  std::size_t m_chunk_index{std::numeric_limits<std::size_t>::max()};

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

    m_logger.log_info("Check guard...");

    if (m_guard.is_active()) {
      m_system_errors.add(type::SystemError::GuardLock);
    }

    m_logger.log_info("{}", m_system_errors.has_any() ? "Not ready" : "Ready");
  }

  auto process_ecu_loop() noexcept -> void {
    type::ECUTelemetry ecu_telemetry{};

    m_system_errors.update(m_ecu.update());
    m_system_errors.update(m_ecu.get_telemetry(ecu_telemetry));

    m_ecu_telemetry.store(ecu_telemetry);
  }

  auto process_ota_loop() noexcept -> void {
    auto const [size, total, index, data] = m_ota_chunk.load();

    if (size == 0 || index == m_chunk_index) {
      return;
    }

    if (!m_ota_manager.isActive()) {
      if (!m_ota_manager.startUpdate(size)) {
        m_system_errors.update(m_ble_manager.send_ota_notify(type::OTAStatus::Error));
        m_chunk_index = std::numeric_limits<std::size_t>::max();
        return;
      }

      m_logger.log_info("Start OTA (Size: {} bytes, Chunks: {})", size, total);
      m_system_state.store(type::SystemState::Update);
    }

    if (!m_ota_manager.writeChunk(data, index, size)) {
      m_logger.log_error("Failed to write chunk {}", index);
      m_system_errors.update(m_ble_manager.send_ota_notify(type::OTAStatus::Error));
      m_system_state.store(type::SystemState::Normal);
      m_chunk_index = std::numeric_limits<std::size_t>::max();
      return;
    }

    m_chunk_index = index;
    m_logger.log_info("Written chunk [{}/{}]", index + 1, total);

    if (index == total - 1) {
      if (!m_ota_manager.endUpdate()) {
        m_logger.log_error("Failed to finalize OTA update");
        m_system_errors.update(m_ble_manager.send_ota_notify(type::OTAStatus::Error));
        m_system_state.store(type::SystemState::Normal);
        m_chunk_index = std::numeric_limits<std::size_t>::max();
        return;
      }

      m_logger.log_info("OTA successfully written. Rebooting...");
      m_system_errors.update(m_ble_manager.send_ota_notify(type::OTAStatus::Completed));

      OTAManager::reboot();

      return;
    }

    m_system_errors.update(m_ble_manager.send_ota_notify(type::OTAStatus::ReadyForNext));
  }

  auto process_system_loop() noexcept -> void {
    m_mode_button.update();
    m_system_errors.update(m_mode_indicator.update());

    type::SystemState const system_state = m_system_state.load();
    type::ECUTelemetry const ecu_telemetry = m_ecu_telemetry.load();

    if (system_state == type::SystemState::Normal)
      if (m_mode_button.is_long_press())
        m_target_speed.store(ecu_telemetry.speed);
  }

  auto process_critical_loop() noexcept -> void {
    type::Control const control = m_control.load();
    type::SystemState const system_state = m_system_state.load();
    type::ECUTelemetry const ecu_telemetry = m_ecu_telemetry.load();

    type::Speed const target_speed = m_target_speed.load();
    type::Speed const current_speed = ecu_telemetry.speed;

    type::Position accelerator_position{0};

    if (system_state == type::SystemState::Normal) {
      m_system_errors.update(m_accelerator.get_position(accelerator_position));
    }

    type::Position const throttle_position = m_logic.calculate_servo_position(accelerator_position, current_speed, target_speed, control);

    m_servo.set_position(throttle_position);

    type::ServoTelemetry servo_telemetry{};
    m_servo.get_telemetry(servo_telemetry);

    type::DriveTelemetry const drive_telemetry{
        .throttle_position = throttle_position,
        .accelerator_position = accelerator_position,
        .servo_telemetry = servo_telemetry,
    };
    m_driver_telemetry.store(drive_telemetry);
  }

  auto process_telemetry_loop() noexcept -> void {
    type::Speed const target_speed = m_target_speed.load();
    type::SystemState const system_state = m_system_state.load();
    type::ECUTelemetry const ecu_telemetry = m_ecu_telemetry.load();

    bool const guard_active = m_guard.is_active();
    bool const brake_active = m_brake.is_active();

    auto const [throttle_position, accelerator_position, servo_telemetry] = m_driver_telemetry.load();

    type::SystemTelemetry const system_telemetry{
        .is_guard_active = guard_active,
        .is_brake_enabled = brake_active,

        .servo_telemetry = servo_telemetry,
        .ecu_telemetry = ecu_telemetry,
        .accelerator_position = accelerator_position,
        .throttle_position = throttle_position,
        .target_speed = target_speed,

        .system_state = system_state,
        .system_errors = m_system_errors.get_all(),
    };
    m_system_errors.update(m_ble_manager.send_telemetry(system_telemetry));
  }
};
