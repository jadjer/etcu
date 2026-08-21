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

#include <concepts>
#include "type.hpp"

namespace concepts {

template <type::CoreId T>
concept CoreId = T <= 1;

template <type::CoreRate T>
concept CoreRate = T >= 10;

template <type::Ticks T, type::Ticks Min, type::Ticks Max>
concept Ticks = T >= Min && T <= Max;

template <typename T>
concept HasStructVersion = std::is_trivially_copyable_v<T> && requires(T data) {
  { data.struct_version } -> std::convertible_to<std::uint32_t>;
  { T::StructName } -> std::convertible_to<char const*>;
};

template <typename T>
concept IsBounded = requires(T instance) {
  requires std::constructible_from<T, std::int32_t>;
  { T::min_value } -> std::convertible_to<std::int32_t>;
  { T::max_value } -> std::convertible_to<std::int32_t>;
  { instance.value } -> std::same_as<std::int32_t&>;
  { instance = std::declval<std::int32_t>() } -> std::same_as<T&>;
};

template <typename T>
concept Accelerator =
    requires(T accelerator, type::AcceleratorCalibrationData calibration_data, type::Position const& position, type::Position& position_result) {
      { accelerator.init() } noexcept -> std::same_as<type::SystemError>;
      { accelerator.set_calibration(calibration_data) } noexcept -> std::same_as<void>;
      { accelerator.calibrate(calibration_data) } noexcept -> std::same_as<type::SystemError>;
      { accelerator.set_minimal_position(position) } noexcept -> std::same_as<void>;
      { accelerator.set_maximal_position(position) } noexcept -> std::same_as<void>;
      { accelerator.get_position(position_result) } noexcept -> std::same_as<type::SystemError>;
    };

template <typename T>
concept Button = requires(T button) {
  { button.init() } noexcept -> std::same_as<type::SystemError>;
  { button.update() } noexcept -> std::same_as<void>;
  { button.is_active() } noexcept -> std::same_as<bool>;
  { button.is_short_press() } noexcept -> std::same_as<bool>;
  { button.is_long_press() } noexcept -> std::same_as<bool>;
};

template <typename T>
concept Switch = requires(T s) {
  { s.init() } noexcept -> std::same_as<type::SystemError>;
  { s.is_active() } noexcept -> std::same_as<bool>;
};

template <typename T>
concept ECU = requires(T ecu, type::ECUTelemetry telemetry) {
  { ecu.init() } noexcept -> std::same_as<type::SystemError>;
  { ecu.update() } noexcept -> std::same_as<type::SystemError>;
  { ecu.get_telemetry(telemetry) } noexcept -> std::same_as<type::SystemError>;
};

template <typename T>
concept Indicator = requires(T indicator) {
  { indicator.init() } noexcept -> std::same_as<type::SystemError>;
  { indicator.update() } noexcept -> std::same_as<type::SystemError>;
};

template <typename T>
concept Servo = requires(T servo, type::Position const& position, type::ServoTelemetry& telemetry) {
  { servo.init() } noexcept -> std::same_as<type::SystemError>;
  { servo.set_position(position) } noexcept -> std::same_as<void>;
  { servo.get_telemetry(telemetry) } noexcept -> std::same_as<bool>;
};

template <typename T>
concept Controller = requires(T controller) {
  { controller.init() } noexcept -> std::same_as<void>;
  { controller.process_system_loop() } noexcept -> std::same_as<void>;
  { controller.process_critical_loop() } noexcept -> std::same_as<void>;
};

template <typename T>
concept GPIO = requires(T gpio, bool const level) {
  { gpio.init() } noexcept -> std::same_as<bool>;
  { gpio.get_level() } noexcept -> std::same_as<bool>;
  { gpio.set_level(level) } noexcept -> std::same_as<bool>;
  { gpio.enable() } noexcept -> std::same_as<bool>;
  { gpio.disable() } noexcept -> std::same_as<bool>;
};

template <typename T>
concept UART = requires(T uart, std::array<type::Byte, 10> buffer, type::Time timeout) {
  { uart.init() } noexcept -> std::same_as<bool>;
  { uart.flush() } noexcept -> std::same_as<bool>;
  { uart.template write<10>(buffer) } noexcept -> std::same_as<bool>;
  { uart.template read<10>(buffer, timeout) } noexcept -> std::same_as<int>;
};

template <typename T, typename V>
concept ADC = requires(T adc, V& voltage) {
  { adc.init() } noexcept -> std::same_as<bool>;
  { adc.template configure_channel<type::ADCChannelId::ADC_CHANNEL_0>() } noexcept -> std::same_as<bool>;
  { adc.template get_voltage<type::ADCChannelId::ADC_CHANNEL_0>(voltage) } noexcept -> std::same_as<bool>;
};

}  // namespace concepts
