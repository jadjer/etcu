//
// Created by jadjer on 23.07.26.
//

#pragma once

#include "types.hpp"

#include <concepts>

namespace concepts {

template <typename T>
concept AcceleratorConcept =
    requires(T a, std::uint16_t& out1, std::uint16_t& out2, SystemError& err) {
      { a.init() } noexcept -> std::same_as<void>;
      { a.read(out1, out2, err) } noexcept -> std::same_as<std::uint16_t>;
    };

template <typename T>
concept ButtonConcept = requires(T b) {
  { b.init() } noexcept -> std::same_as<void>;
  { b.update() } noexcept -> std::same_as<void>;
  { b.is_short_press() } noexcept -> std::same_as<bool>;
  { b.is_long_press() } noexcept -> std::same_as<bool>;
};

template <typename T>
concept ECUConcept = requires(T e, Mode mode, std::uint16_t pos, SystemError err) {
  { e.init() } noexcept -> std::same_as<void>;
  // { e.update(mode, pos, err) } -> std::same_as<void>;
};

template <typename T>
concept IndicatorConcept = requires(T i, Mode mode, SystemError err) {
  { i.init() } noexcept -> std::same_as<void>;
  { i.set_status(mode, err) } noexcept -> std::same_as<void>;
};

template <typename T>
concept ServoConcept = requires(T s, std::uint16_t position, SystemError err) {
  { s.init() } noexcept -> std::same_as<void>;
  { s.set_position(position, err) } noexcept -> std::same_as<bool>;
  // { s.read_telemetry(std::uint16_t) } -> std::same_as<void>;
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
