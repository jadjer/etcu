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
#include "controller_cruise.hpp"
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
  static constexpr type::ServoCalibrationData servo_calibration_factory{
      .position{.min = 600, .max = 1250},
  };
  static constexpr type::AcceleratorCalibrationData accelerator_calibration_factory{
      .hall_a{.min = 650, .max = 1350},
      .hall_b{.min = 320, .max = 690},
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

  common::AtomicContainer<type::Control> m_control{};
  common::AtomicContainer<DriveTelemetry> m_driver_telemetry{};
  common::AtomicContainer<type::Calibration> m_calibration{};
  common::AtomicContainer<type::SystemState> m_system_state{type::SystemState::Normal};
  common::AtomicContainer<type::ECUTelemetry> m_ecu_telemetry{};
  common::AtomicContainer<type::OTAChunk<constants::bluetooth::OTAPayloadSize>> m_ota_chunk{};

  Logger m_logger;
  Storage m_storage;
  OTAManager m_ota_manager;
  ControllerCruise m_cruise;
  SystemErrors m_system_errors;

  std::size_t m_chunk_index{std::numeric_limits<std::size_t>::max()};
  type::Control m_last_control{};
  type::Calibration m_last_calibration{};
  bluetooth::BLEManager m_ble_manager{m_control, m_calibration, m_ota_chunk};

  [[nodiscard]] auto validate_cruise_activation(type::Control const& control,
                                                type::RPM const& current_rpm,
                                                type::Speed const& current_speed,
                                                type::Speed const& target_speed,
                                                bool const is_safety_active,
                                                const char* context) noexcept -> bool {
    if (is_safety_active) {
      m_logger.log_info("Cruise %s error: safety active", context);
      return false;
    }

    if (!control.cruise.rpm.contains(current_rpm)) {
      m_logger.log_info("Cruise %s error: RPM out of range", context);
      return false;
    }

    if (!control.cruise.speed.contains(current_speed)) {
      m_logger.log_info("Cruise %s error: Current Speed (%d km/h) out of range", context, current_speed.get());
      return false;
    }

    if (!control.cruise.speed.contains(target_speed)) {
      m_logger.log_info("Cruise %s error: Target Speed (%d km/h) out of range", context, target_speed.get());
      return false;
    }

    return true;
  }

  auto handle_mode_button(std::uint16_t const pattern,
                          type::Control& control,
                          type::Speed const& current_speed,
                          type::RPM const& current_rpm,
                          bool const is_safety_active) -> void {
    auto const update_servo_max = [&](std::int16_t const max_position) {
      control.servo.max = type::Position{max_position};
      m_control.store(control);
      m_indicator.blink_times(1);
      m_logger.log_info("Set servo max as %d", max_position);
      m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.send_control(control));
    };

    bool const is_cruise_enabled = m_cruise.is_enable();
    bool const is_cruise_active = m_cruise.is_active();

    switch (pattern) {
      case device::BuildPattern(device::ClickType::Short): {
        if (!is_cruise_enabled) {
          m_logger.log_info("Cruise not enabled");
          m_indicator.blink_times(2);
          return;
        }

        if (is_cruise_active && m_cruise.pause()) {
          m_indicator.turn_off();
          m_logger.log_info("Cruise paused");
          return;
        }

        type::Speed const target_speed = m_cruise.get_target_speed();

        if (validate_cruise_activation(control, current_rpm, current_speed, target_speed, is_safety_active, "resume") && m_cruise.resume()) {
          m_indicator.turn_on();
          m_logger.log_info("Cruise resumed at: %d km/h", target_speed.get());
          return;
        }

        m_indicator.blink_times(2);
        return;
      }

      case device::BuildPattern(device::ClickType::Short, device::ClickType::Short): {
        if (!is_cruise_enabled) {
          m_logger.log_info("Cruise not enabled");
          m_indicator.blink_times(2);
          return;
        }

        m_cruise.disable();
        m_indicator.turn_off();
        m_indicator.blink_times(1);
        m_logger.log_info("Cruise deactivated");
        return;
      }

      case device::BuildPattern(device::ClickType::Long): {
        if (validate_cruise_activation(control, current_rpm, current_speed, current_speed, is_safety_active, "enable")) {
          m_cruise.enable();
          m_indicator.turn_on();
          m_logger.log_info("Cruise enabled and set at: %d km/h", current_speed.get());
          return;
        }

        m_indicator.blink_times(2);
        return;
      }

      case device::BuildPattern(device::ClickType::Long, device::ClickType::Short):
        update_servo_max(300);
        return;

      case device::BuildPattern(device::ClickType::Long, device::ClickType::Short, device::ClickType::Short):
        update_servo_max(600);
        return;

      case device::BuildPattern(device::ClickType::Long, device::ClickType::Short, device::ClickType::Short, device::ClickType::Short):
        update_servo_max(900);
        return;

      default:
        m_indicator.blink_times(2);
        break;
    }
  }

  [[nodiscard]] static auto check_safety_active(type::ECUTelemetry const& ecu, type::Control const& control, bool const is_brake_active) noexcept -> bool {
    auto const& cruise = control.cruise;

    return !cruise.rpm.contains(ecu.rpm) || !cruise.speed.contains(ecu.speed) || ecu.is_neutral || is_brake_active;
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

    if (!m_storage.init()) [[unlikely]] {
      m_logger.log_error("Storage init fault");
      return false;
    }

    if (!m_cruise.init()) [[unlikely]] {
      m_logger.log_error("Cruise init fault");
      return false;
    }

    m_system_errors.update(type::ErrorMaskECU, m_ecu.init());
    m_system_errors.update(type::ErrorMaskServo, m_servo.init());
    m_system_errors.update(type::ErrorMaskPeripheral, m_brake.init());
    m_system_errors.update(type::ErrorMaskGuard, m_guard.init());
    m_system_errors.update(type::ErrorMaskPeripheral, m_mode_button.init());
    m_system_errors.update(type::ErrorMaskAccelerator, m_accelerator.init());
    m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.init());
    m_system_errors.update(type::ErrorMaskIndicator, m_indicator.init());

    if (m_system_errors.has(type::ErrorMaskServo)) {
      return m_logger.log_error("Servo init fault"), false;
    }

    if (m_system_errors.has(type::ErrorMaskAccelerator)) {
      return m_logger.log_error("Accelerator init fault"), false;
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
        m_storage.save(factory_calib);
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

    if (type::ECUTelemetry ecu_telemetry; m_ecu.get_telemetry(ecu_telemetry)) [[likely]] {
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
      m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.send_ota_status(type::OTAStatus::Error));
      m_logger.log_error("System not ready for update. Abort update");
      return;
    }

    auto handle_error = [this](const char* msg) {
      m_logger.log_error(msg);
      m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.send_ota_status(type::OTAStatus::Error));
      m_system_state.store(type::SystemState::Normal);
    };

    if (!m_ota_manager.is_active()) {
      if (!m_ota_manager.start_update(size)) {
        return handle_error("Start OTA error");
      }

      m_logger.log_info("Start OTA (Size: %d bytes)", size);
      m_system_state.store(type::SystemState::Update);
    }

    if (!m_ota_manager.write_chunk(index, data)) {
      return handle_error("Failed to write chunk");
    }

    m_chunk_index = index;
    m_logger.log_info("Written chunk [%d/%d]", index + 1, total);

    if (index == total - 1) {
      if (!m_ota_manager.end_update()) {
        return handle_error("Failed to finalize OTA update");
      }

      m_logger.log_info("OTA written. Rebooting...");
      m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.send_ota_status(type::OTAStatus::Completed));

      OTAManager::reboot();

      return;
    }

    m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.send_ota_status(type::OTAStatus::ReadyForNext));
  }

  auto process_system_loop() noexcept -> void {
    m_mode_button.update();
    m_indicator.update();

    if (m_guard.is_active()) [[unlikely]] {
      m_system_errors.update(type::ErrorMaskGuard, type::SystemError::GuardLock);
      m_system_state.store(type::SystemState::Off);
      m_logger.log_warn("Guard active. System shutdown");
    }

    type::SystemState const system_state = m_system_state.load();

    if (system_state == type::SystemState::Off) {
      if (m_cruise.is_active()) {
        m_cruise.disable();
        m_indicator.turn_off();
      }
      return;
    }

    type::ECUTelemetry const ecu = m_ecu_telemetry.load();
    type::Control control = m_control.load();
    bool const is_brake_active = m_brake.is_active();

    bool const is_safety_active = check_safety_active(ecu, control, is_brake_active);
    if (system_state == type::SystemState::Normal && m_mode_button.has_event()) {
      handle_mode_button(m_mode_button.get_pattern(), control, ecu.speed, ecu.rpm, is_safety_active);
    }

    if (m_cruise.is_active()) {
      m_indicator.turn_on();
    } else {
      m_indicator.turn_off();
    }
  }

  auto process_control_loop() noexcept -> void {
    type::Control const control = m_control.load();

    if (control == m_last_control) {
      return;
    }

    if (!m_storage.save(control)) {
      m_logger.log_error("Control data save error");
    }

    m_last_control = control;
    m_logger.log_info("Control updated");
  }

  auto process_critical_loop() noexcept -> void {
    if (type::SystemState const system_state = m_system_state.load(); system_state != type::SystemState::Normal) [[unlikely]] {
      m_system_errors.update(type::ErrorMaskServo, m_servo.set_position(type::Position{0}));
      return;
    }

    type::AcceleratorTelemetry accelerator_telemetry;
    m_system_errors.update(type::ErrorMaskAccelerator, m_accelerator.get_telemetry(accelerator_telemetry));

    type::Control const control = m_control.load();
    type::ECUTelemetry const ecu = m_ecu_telemetry.load();
    bool const is_brake_active = m_brake.is_active();
    bool const is_safety_active = check_safety_active(ecu, control, is_brake_active);

    type::Position const throttle_position = m_cruise.generate_throttle_position(accelerator_telemetry.position, ecu.speed, ecu.rpm, is_safety_active, control);
    m_system_errors.update(type::ErrorMaskServo, m_servo.set_position(throttle_position));

    type::ServoTelemetry servo_telemetry;
    m_system_errors.update(type::ErrorMaskServo, m_servo.get_telemetry(servo_telemetry));

    m_driver_telemetry.store(DriveTelemetry{
        .throttle_position = throttle_position,
        .servo_telemetry = servo_telemetry,
        .accelerator_telemetry = accelerator_telemetry,
    });
  }

  auto process_telemetry_loop() noexcept -> void {
    auto const [throttle_position, servo_telemetry, accelerator_telemetry] = m_driver_telemetry.load();

    type::ECUTelemetry const ecu_telemetry = m_ecu_telemetry.load();

    type::CruiseTelemetry const cruise_telemetry{
        .is_enabled = m_cruise.is_enable(),
        .is_activated = m_cruise.is_active(),

        .error = m_cruise.get_error(),
        .correction = m_cruise.get_correction(),

        .target_speed = m_cruise.get_target_speed(),
        .current_speed = ecu_telemetry.speed,

        .base_position = m_cruise.get_target_position(),
        .current_position = throttle_position,
    };

    type::SystemTelemetry const system_telemetry{
        .is_guard_active = m_guard.is_active(),
        .is_brake_enabled = m_brake.is_active(),

        .throttle_position = throttle_position,

        .ecu_telemetry = ecu_telemetry,
        .servo_telemetry = servo_telemetry,
        .cruise_telemetry = cruise_telemetry,
        .accelerator_telemetry = accelerator_telemetry,

        .system_state = m_system_state.load(),
        .system_errors = m_system_errors.get_error_mask(),
    };

    m_system_errors.update(type::ErrorMaskBluetooth, m_ble_manager.send_telemetry(system_telemetry));
  }

  auto process_calibration_loop() noexcept -> void {
    type::Calibration const calibration = m_calibration.load();

    if (calibration == m_last_calibration) {
      return;
    }

    auto update_calibration = [this](auto& current, auto& last, auto& device, const char* name) {
      if (current == last) {
        return;
      }

      if (!m_storage.save(current)) {
        return m_logger.log_error("%s calibration set error", name);
      }

      device.set_calibration(current);
      last = current;

      m_logger.log_info("%s calibration set", name);
    };

    update_calibration(calibration.servo, m_last_calibration.servo, m_servo, "Servo");
    update_calibration(calibration.accelerator, m_last_calibration.accelerator, m_accelerator, "Accelerator");
  }
};
