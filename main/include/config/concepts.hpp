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
#include "type/telemetry.hpp"

namespace concepts {

template <typename T>
concept GPIO = requires(T const const_gpio, bool const level) {
  { const_gpio.init() } noexcept -> std::same_as<bool>;
  { const_gpio.get_level() } noexcept -> std::same_as<bool>;
  { const_gpio.set_level(level) } noexcept -> std::same_as<bool>;
  { const_gpio.enable() } noexcept -> std::same_as<bool>;
  { const_gpio.disable() } noexcept -> std::same_as<bool>;
};

template <typename T>
concept UART = requires(T uart, std::array<std::uint8_t, 1> buffer, type::primitive::Time timeout) {
  { uart.init() } noexcept -> std::same_as<bool>;
  { uart.deinit() } noexcept -> std::same_as<bool>;
  { uart.flush() } noexcept -> std::same_as<bool>;
  { uart.template write<1>(buffer) } noexcept -> std::same_as<bool>;
  { uart.template read<1>(buffer, timeout) } noexcept -> std::same_as<int>;
};

template <typename T>
concept ADC = requires(T adc, T const const_adc, std::uint16_t& voltage) {
  { adc.init() } noexcept -> std::same_as<bool>;
  { adc.template configure_channel<0>() } noexcept -> std::same_as<bool>;
  { const_adc.template get_value<0>(voltage) } noexcept -> std::same_as<bool>;
  { const_adc.template get_voltage<0>(voltage) } noexcept -> std::same_as<bool>;
};

}  // namespace concepts
