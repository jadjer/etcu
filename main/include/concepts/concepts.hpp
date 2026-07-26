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
#include "types.hpp"

namespace concepts {

template <CoreId T>
concept CoreIdConcept = (T <= 1);

template <CoreRate T>
concept CoreRateConcept = (T >= 10);

template <Ticks T, Ticks Min, Ticks Max>
concept TicksConcept = (T >= Min && T <= Max);

template <typename T>
concept HasStructVersion = requires(T data) {
  { data.struct_version } -> std::same_as<std::uint32_t&>;
};

template <typename T>
concept AcceleratorConcept = requires(T a, Position position, Position& position_result) {
  { a.init() } noexcept -> std::same_as<SystemError>;
  { a.calibrate() } noexcept -> std::same_as<SystemError>;
  { a.set_minimal_position(position) } noexcept -> std::same_as<void>;
  { a.set_maximal_position(position) } noexcept -> std::same_as<void>;
  { a.get_position(position_result) } noexcept -> std::same_as<SystemError>;
};

template <typename T>
concept ButtonConcept = requires(T b, Ticks ticks) {
  { b.init() } noexcept -> std::same_as<SystemError>;
  { b.update() } noexcept -> std::same_as<void>;
  { b.is_active() } noexcept -> std::same_as<bool>;
  { b.is_short_press() } noexcept -> std::same_as<bool>;
  { b.is_long_press() } noexcept -> std::same_as<bool>;
  { b.is_long_then(ticks) } noexcept -> std::same_as<bool>;
};

template <typename T>
concept ECUConcept = requires(T e) {
  { e.init() } noexcept -> std::same_as<SystemError>;
  { e.update() } noexcept -> std::same_as<SystemError>;
  { e.get_rpm() } noexcept -> std::same_as<RPM>;
  { e.get_tps() } noexcept -> std::same_as<MilliVolt>;
  { e.get_speed() } noexcept -> std::same_as<Speed>;
};

template <typename T>
concept IndicatorConcept = requires(T i, Mode mode, SystemError err) {
  { i.init() } noexcept -> std::same_as<SystemError>;
  { i.update() } noexcept -> std::same_as<SystemError>;
  { i.set_status(mode) } noexcept -> std::same_as<SystemError>;
};

template <typename T>
concept ServoConcept = requires(T s, Position position, ServoTelemetry& telemetry) {
  { s.init() } noexcept -> std::same_as<SystemError>;
  { s.self_test() } noexcept -> std::same_as<SystemError>;
  { s.set_position(position) } noexcept -> std::same_as<SystemError>;
  { s.read_telemetry(telemetry) } noexcept -> std::same_as<SystemError>;
};

template <typename T>
concept ControllerConcept = requires(T c) {
  { c.init() } noexcept -> std::same_as<void>;
  { c.process_system_loop() } noexcept -> std::same_as<void>;
  { c.process_critical_loop() } noexcept -> std::same_as<void>;
};

template <typename T>
concept LoggerConcept = requires(T l, const char* msg) {
  { l.init() } noexcept -> std::same_as<void>;
  { l.log_info(msg) } noexcept -> std::same_as<void>;
  { l.log_warn(msg) } noexcept -> std::same_as<void>;
  { l.log_error(msg) } noexcept -> std::same_as<void>;
};

}  // namespace concepts
