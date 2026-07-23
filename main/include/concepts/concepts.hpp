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

template <typename T>
concept AcceleratorConcept = requires(T a, SystemError& err) {
  { a.init() } noexcept -> std::same_as<void>;
  { a.get_position(err) } noexcept -> std::same_as<std::uint16_t>;
};

template <typename T>
concept ButtonConcept = requires(T b) {
  { b.init() } noexcept -> std::same_as<void>;
  { b.update() } noexcept -> std::same_as<void>;
  { b.is_short_press() } noexcept -> std::same_as<bool>;
  { b.is_long_press() } noexcept -> std::same_as<bool>;
};

template <typename T>
concept ECUConcept = requires(T e) {
  { e.init() } noexcept -> std::same_as<void>;
  { e.update() } noexcept -> std::same_as<void>;
  { e.get_rpm() } noexcept -> std::same_as<std::uint16_t>;
  { e.get_tps() } noexcept -> std::same_as<std::uint16_t>;
  { e.get_speed() } noexcept -> std::same_as<std::uint16_t>;
};

template <typename T>
concept IndicatorConcept = requires(T i, Mode mode, SystemError err) {
  { i.init() } noexcept -> std::same_as<void>;
  { i.set_status(mode, err) } noexcept -> std::same_as<void>;
};

template <typename T>
concept ServoConcept = requires(T s, std::uint16_t const position, ServoTelemetry& telemetry, SystemError& err) {
  { s.init() } noexcept -> std::same_as<void>;
  { s.set_position(position, err) } noexcept -> std::same_as<bool>;
  { s.read_telemetry(telemetry, err) } noexcept -> std::same_as<bool>;
};

template <typename T>
concept SwitchConcept = requires(T s) {
  { s.init() } noexcept -> std::same_as<void>;
  { s.is_active() } noexcept -> std::same_as<bool>;
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
