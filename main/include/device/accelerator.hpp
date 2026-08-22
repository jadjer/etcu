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

#include <algorithm>
#include "common/map_range.hpp"
#include "concepts.hpp"
#include "type.hpp"

namespace device {

template <class Driver, type::ADCChannelId hallA, type::ADCChannelId hallB, type::Position threshold>
  requires concepts::ADC<Driver, type::MilliVolt>

class Accelerator {
  Driver& m_driver_adc;

  type::Position m_current_min_position{type::Position::MIN_VALUE};
  type::Position m_current_max_position{type::Position::MAX_VALUE};

  type::AcceleratorCalibrationData m_calibration_data{
      .hall_a_minimal = type::MilliVolt{650},
      .hall_a_maximal = type::MilliVolt{1350},
      .hall_b_minimal = type::MilliVolt{320},
      .hall_b_maximal = type::MilliVolt{690},
  };

 public:
  constexpr explicit Accelerator(Driver& driver_adc) noexcept : m_driver_adc(driver_adc) {}

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_driver_adc.init())
      return type::SystemError::AcceleratorInitFault;

    if (!m_driver_adc.template configure_channel<hallA>())
      return type::SystemError::AcceleratorInitFault;

    if (!m_driver_adc.template configure_channel<hallB>())
      return type::SystemError::AcceleratorInitFault;

    return type::SystemError::None;
  }

  auto set_calibration(type::AcceleratorCalibrationData const& calibration_data) noexcept -> void { m_calibration_data = calibration_data; }

  [[nodiscard]] auto calibrate(type::AcceleratorCalibrationData& calibration_data) noexcept -> type::SystemError {
    type::MilliVolt voltage_a{0};
    type::MilliVolt voltage_b{0};

    if (!m_driver_adc.template get_voltage<hallA>(voltage_a))
      return type::SystemError::AcceleratorReadFault;

    if (!m_driver_adc.template get_voltage<hallB>(voltage_b))
      return type::SystemError::AcceleratorReadFault;

    auto const v_a = voltage_a.value;
    auto const v_b = voltage_b.value;

    auto const a_min = m_calibration_data.hall_a_minimal.value;
    auto const a_max = m_calibration_data.hall_a_maximal.value;
    auto const b_min = m_calibration_data.hall_b_minimal.value;
    auto const b_max = m_calibration_data.hall_b_maximal.value;

    m_calibration_data.hall_a_minimal = type::MilliVolt{std::min(a_min, v_a)};
    m_calibration_data.hall_a_maximal = type::MilliVolt{std::max(a_max, v_a)};
    m_calibration_data.hall_b_minimal = type::MilliVolt{std::min(b_min, v_b)};
    m_calibration_data.hall_b_maximal = type::MilliVolt{std::max(b_max, v_b)};

    calibration_data = m_calibration_data;

    return type::SystemError::None;
  }

  auto set_minimal_position(type::Position const position) noexcept -> void { m_current_min_position = position; }

  auto set_maximal_position(type::Position const position) noexcept -> void { m_current_max_position = position; }

  [[nodiscard]] auto get_position(type::Position& current_position) const noexcept -> type::SystemError {
    type::MilliVolt voltage_a{0};
    type::MilliVolt voltage_b{0};

    if (!m_driver_adc.template get_voltage<hallA>(voltage_a))
      return type::SystemError::AcceleratorReadFault;

    if (!m_driver_adc.template get_voltage<hallB>(voltage_b))
      return type::SystemError::AcceleratorReadFault;

    type::Position const pos_a =
        common::map_range(voltage_a, m_calibration_data.hall_a_minimal, m_calibration_data.hall_a_maximal, m_current_min_position, m_current_max_position);
    type::Position const pos_b =
        common::map_range(voltage_b, m_calibration_data.hall_b_minimal, m_calibration_data.hall_b_maximal, m_current_min_position, m_current_max_position);

    ESP_LOGI("ACC", "%d %d %d %d", voltage_a, voltage_b, pos_a, pos_b);

    auto const raw_diff = std::abs(pos_a.value - pos_b.value);

    if (raw_diff > threshold) [[unlikely]] {
      current_position = type::Position{0};

      return type::SystemError::AcceleratorMismatch;
    }

    current_position = pos_a;

    return type::SystemError::None;
  }
};

}  // namespace device
