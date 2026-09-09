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
#include "type/control.hpp"
#include "type/ota.hpp"
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
concept IndicatorConcept = requires(T indicator, std::size_t const count) {
  { indicator.init() } noexcept -> std::same_as<type::SystemError>;
  { indicator.update() } noexcept -> std::same_as<bool>;
  { indicator.turn_on() } noexcept -> std::same_as<bool>;
  { indicator.turn_off() } noexcept -> std::same_as<bool>;
  { indicator.blink_times(count) } noexcept -> std::same_as<bool>;
};

template <typename T>
concept ServoConcept = requires(T servo, type::ServoCalibrationData const& calibration_data, type::Position const& position, type::ServoTelemetry& telemetry) {
  { servo.init() } noexcept -> std::same_as<type::SystemError>;
  { servo.set_calibration(calibration_data) } noexcept -> std::same_as<void>;
  { servo.set_position(position) } noexcept -> std::same_as<type::SystemError>;
  { servo.get_telemetry(telemetry) } noexcept -> std::same_as<type::SystemError>;
};

template <class Accelerator, class Servo, class ECU, class ModeButton, class Indicator, class Brake, class Guard>
  requires AcceleratorConcept<Accelerator> && ServoConcept<Servo> && ECUConcept<ECU> && ButtonConcept<ModeButton> && IndicatorConcept<Indicator> &&
           SwitchConcept<Brake> && SwitchConcept<Guard>
class Controller {
  static constexpr type::Speed cruise_minimal_speed{40};
  static constexpr type::ServoCalibrationData servo_calibration_factory{
      .position_minimal{600},
      .position_maximal{1250},
  };
  static constexpr type::AcceleratorCalibrationData accelerator_calibration_factory{
      .hall_a_minimal{650},
      .hall_a_maximal{1350},
      .hall_b_minimal{320},
      .hall_b_maximal{690},
  };

  struct DriveTelemetry {
    type::Position throttle_position{};
    type::ServoTelemetry servo_telemetry{};
    type::AcceleratorTelemetry accelerator_telemetry{};
  };

  ECU& m_ecu;
  Servo& m_servo;
  Brake& m_brake;
  Guard& m_guard;
  Indicator& m_indicator;
  ModeButton& m_mode_button;
  Accelerator& m_accelerator;

  common::AtomicContainer<type::Speed> m_target_speed{0};
  common::AtomicContainer<type::Control> m_control{};
  common::AtomicContainer<type::Calibration> m_calibration{};
  common::AtomicContainer<type::SystemState> m_system_state{type::SystemState::Normal};
  common::AtomicContainer<type::ECUTelemetry> m_ecu_telemetry{};
  common::AtomicContainer<type::OTAChunk<constants::bluetooth::OTAPayloadSize>> m_ota_chunk{};

  common::AtomicContainer<DriveTelemetry> m_driver_telemetry{};

  Logger m_logger;
  Storage m_storage;
  ControllerLogic m_logic;
  OTAManager m_ota_manager;
  std::size_t m_chunk_index{std::numeric_limits<std::size_t>::max()};
  SystemErrors m_system_errors;
  type::Control m_last_control{};
  type::Calibration m_last_calibration{};
  bluetooth::BLEManager m_ble_manager{m_control, m_calibration, m_ota_chunk};

  auto handle_mode_button(std::uint16_t const pattern, type::Control& control, type::ECUTelemetry const& ecu_telemetry, bool const safety_active) -> void {
    if (pattern == device::BuildPattern(device::ClickType::Short)) {
      m_target_speed.store(0);
      m_indicator.turn_off();
      return;
    }

    if (pattern == device::BuildPattern(device::ClickType::Short, device::ClickType::Short)) {
      if (ecu_telemetry.speed < cruise_minimal_speed) {
        m_indicator.blink_times(2);
        return;
      }

      if (safety_active) {
        m_indicator.blink_times(3);
        return;
      }

      m_target_speed.store(ecu_telemetry.speed);
      m_indicator.turn_on();

      return;
    }

    type::Position max_servo{0};
    if (pattern == device::BuildPattern(device::ClickType::Long, device::ClickType::Short)) {
      max_servo = 300;
    } else if (pattern == device::BuildPattern(device::ClickType::Long, device::ClickType::Short, device::ClickType::Short)) {
      max_servo = 600;
    } else if (pattern == device::BuildPattern(device::ClickType::Long, device::ClickType::Short, device::ClickType::Short, device::ClickType::Short)) {
      max_servo = 900;
    }

    if (max_servo > 0) {
      control.servo.max = max_servo;
      m_control.store(control);
      m_indicator.blink_times(1);
    }
  }

 public:
  constexpr explicit Controller(Accelerator& accelerator,
                                Servo& servo,
                                ECU& ecu,
                                ModeButton& mode_button,
                                Indicator& indicator,
                                Brake& brake,
                                Guard& guard) noexcept
      : m_ecu(ecu), m_servo(servo), m_brake(brake), m_guard(guard), m_indicator(indicator), m_mode_button(mode_button), m_accelerator(accelerator) {}

  constexpr Controller() noexcept = delete;

  Controller(Controller const&) noexcept = delete;
  auto operator=(Controller const&) noexcept -> Controller& = delete;

  Controller(Controller&&) noexcept = delete;
  auto operator=(Controller&&) noexcept -> Controller& = delete;

  constexpr ~Controller() noexcept = default;

  auto init() noexcept -> bool {
    Logger::init();
    m_logger.log_info("Initialization...");

    struct InitStep {
      type::SystemError mask;
      type::SystemError error;
    };

    InitStep const steps[] = {{.mask = type::ErrorMaskECU, .error = m_ecu.init()},
                              {.mask = type::ErrorMaskServo, .error = m_servo.init()},
                              {.mask = type::ErrorMaskPeripheral, .error = m_brake.init()},
                              {.mask = type::ErrorMaskGuard, .error = m_guard.init()},
                              {.mask = type::ErrorMaskPeripheral, .error = m_mode_button.init()},
                              {.mask = type::ErrorMaskAccelerator, .error = m_accelerator.init()},
                              {.mask = type::ErrorMaskBluetooth, .error = m_ble_manager.init()},
                              {.mask = type::ErrorMaskIndicator, .error = m_indicator.init()}};

    for (auto const& step : steps) {
      m_system_errors.update(step.mask, step.error);
    }

    struct CriticalCheck {
      type::SystemError mask;
      char const* error_msg;
    };

    CriticalCheck const critical_checks[] = {{.mask = type::ErrorMaskServo, .error_msg = "Servo init fault"},
                                             {.mask = type::ErrorMaskAccelerator, .error_msg = "Accelerator init fault"}};

    for (auto const& check : critical_checks) {
      if (m_system_errors.has(check.mask)) [[unlikely]] {
        m_logger.log_error(check.error_msg);
        return false;
      }
    }

    if (!m_storage.init()) [[unlikely]] {
      m_logger.log_error("Storage init fault");
      return false;
    }

    m_logger.log_info("Initialization...Done");

    return true;
  }

  auto configure() noexcept -> bool {
    m_logger.log_info("Configuration...");

    if (m_storage.load(m_last_control)) {
      m_control.store(m_last_control);
    }

    auto load_or_factory = [this](auto& out_calib, auto const& factory_calib, auto& device, const char* name) {
      auto temp = factory_calib;

      if (!m_storage.load(temp)) {
        m_logger.log_warn("%s factory calibration loading...", name);
        if (m_storage.save(factory_calib)) {
          m_logger.log_warn("%s factory calibration loading...Done", name);
        } else {
          m_logger.log_error("%s factory calibration loading...Error", name);
        }
      }

      device.set_calibration(temp);
      out_calib = temp;
    };

    load_or_factory(m_last_calibration.accelerator, accelerator_calibration_factory, m_accelerator, "Accelerator");
    load_or_factory(m_last_calibration.servo, servo_calibration_factory, m_servo, "Servo");

    m_calibration.store(m_last_calibration);

    m_logger.log_info("Configuration...Done");

    return true;
  }

  auto process_ecu_loop() noexcept -> void {
    m_system_errors.update(type::ErrorMaskECU, m_ecu.update());

    if (type::ECUTelemetry ecu_telemetry; m_ecu.get_telemetry(ecu_telemetry)) {
      m_ecu_telemetry.store(ecu_telemetry);
    }
  }

  auto process_ota_loop() noexcept -> void {
    type::SystemState const state = m_system_state.load();

    auto const [size, total, index, data] = m_ota_chunk.load();

    if (size == 0 || index == m_chunk_index) {
      return;
    }

    if (state != type::SystemState::Normal && state != type::SystemState::Update) {
      return;
    }

    auto handle_error = [this](const char* log_msg, auto... args) {
      if (log_msg) {
        m_logger.log_error(log_msg, args...);
      }

      m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.send_ota_notify(type::OTAStatus::Error));
      m_system_state.store(type::SystemState::Normal);
      m_chunk_index = std::numeric_limits<std::size_t>::max();
    };

    if (!m_ota_manager.is_active()) {
      if (state != type::SystemState::Normal) {
        return handle_error("System state is not NORMAL");
      }

      if (!m_ota_manager.start_update(size)) {
        return handle_error("Start OTA error");
      }

      m_logger.log_info("Start OTA (Size: %d bytes, Chunks: %d)", size, total);
      m_system_state.store(type::SystemState::Update);
    }

    if (!m_ota_manager.write_chunk(index, data)) {
      return handle_error("Failed to write chunk %d", index);
    }

    m_chunk_index = index;
    m_logger.log_info("Written chunk [%d/%d]", index + 1, total);

    if (index == total - 1) {
      if (!m_ota_manager.end_update()) {
        return handle_error("Failed to finalize OTA update");
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
    m_indicator.update();

    if (m_guard.is_active()) {
      m_system_errors.update(type::ErrorMaskGuard, type::SystemError::GuardLock);
      m_system_state.store(type::SystemState::Off);
    }

    type::Control control = m_control.load();
    type::Calibration const calibration = m_calibration.load();
    type::SystemState const system_state = m_system_state.load();
    type::ECUTelemetry const ecu_telemetry = m_ecu_telemetry.load();

    bool const safety_active = m_brake.is_active() || ecu_telemetry.is_neutral;

    if (control != m_last_control && !m_storage.save(control)) [[unlikely]] {
      m_logger.log_error("Control data save error");
    }

    auto update_calibration = [this](auto& current, auto& last, auto& device, auto const& name) {
      if (current == last) {
        return;
      }

      if (!m_storage.save(current)) {
        m_logger.log_error("%s calibration set error", name);
      } else {
        m_logger.log_info("%s calibration set", name);
        device.set_calibration(current);
        last = current;
      }
    };

    if (calibration != m_last_calibration) {
      update_calibration(calibration.accelerator, m_last_calibration.accelerator, m_accelerator, "Accelerator");
      update_calibration(calibration.servo, m_last_calibration.servo, m_servo, "Servo");
    }

    if (system_state == type::SystemState::Normal && m_mode_button.has_event()) {
      handle_mode_button(m_mode_button.get_pattern(), control, ecu_telemetry, safety_active);
    }

    if (system_state == type::SystemState::Off || safety_active) {
      m_target_speed.store(0);
      m_indicator.turn_off();
    }
  }

  auto process_critical_loop() noexcept -> void {
    if (type::SystemState const system_state = m_system_state.load(); system_state != type::SystemState::Normal) {
      m_system_errors.update(type::ErrorMaskServo, m_servo.set_position(0));
      return;
    }

    type::Control const control = m_control.load();
    type::Speed const target_speed = m_target_speed.load();
    type::ECUTelemetry const ecu_telemetry = m_ecu_telemetry.load();

    type::AcceleratorTelemetry accelerator_telemetry;
    m_system_errors.update(type::ErrorMaskAccelerator, m_accelerator.get_telemetry(accelerator_telemetry));

    type::Position const servo_position = m_logic.calculate_servo_position(accelerator_telemetry.position, ecu_telemetry.speed, target_speed, control);
    m_system_errors.update(type::ErrorMaskServo, m_servo.set_position(servo_position));

    type::ServoTelemetry servo_telemetry;
    m_system_errors.update(type::ErrorMaskServo, m_servo.get_telemetry(servo_telemetry));

    m_driver_telemetry.store(DriveTelemetry{
        .throttle_position = servo_position,
        .servo_telemetry = servo_telemetry,
        .accelerator_telemetry = accelerator_telemetry,
    });
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
