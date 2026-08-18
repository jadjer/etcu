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

template <typename DriverChannelA, typename DriverChannelB, type::Position threshold>
  requires concepts::ADC<DriverChannelA, type::MilliVolt> && concepts::ADC<DriverChannelB, type::MilliVolt>

class Accelerator {
  DriverChannelA& m_driver_channel_a;
  DriverChannelB& m_driver_channel_b;

  type::Position m_current_min_position{0};
  type::Position m_current_max_position{100};

  type::AcceleratorCalibrationData m_calibration_data{
      .hall_a_minimal = 620,
      .hall_a_maximal = 1320,
      .hall_b_minimal = 220,
      .hall_b_maximal = 460,
  };

 public:
  explicit Accelerator(DriverChannelA& driver_channel_a, DriverChannelB& driver_channel_b) noexcept
      : m_driver_channel_a(driver_channel_a), m_driver_channel_b(driver_channel_b) {}

  auto init() noexcept -> type::SystemError {
    if (!m_driver_channel_a.init())
      return type::SystemError::AcceleratorInitFault;

    if (!m_driver_channel_b.init())
      return type::SystemError::AcceleratorInitFault;

    return type::SystemError::None;
  }

  auto set_calibration(type::AcceleratorCalibrationData const& calibration_data) noexcept -> void { m_calibration_data = calibration_data; }

  auto calibrate(type::AcceleratorCalibrationData& calibration_data) noexcept -> type::SystemError {
    type::MilliVolt constexpr voltage_minimal = 0;
    type::MilliVolt constexpr voltage_maximal = 3300;

    type::MilliVolt voltage_a = 0, voltage_b = 0;

    if (!m_driver_channel_a.get_voltage(voltage_a))
      return type::SystemError::AcceleratorReadFault;

    if (!m_driver_channel_b.get_voltage(voltage_b))
      return type::SystemError::AcceleratorReadFault;

    if (m_calibration_data.hall_a_minimal == 0 && m_calibration_data.hall_a_maximal == 0) {
      m_calibration_data.hall_a_minimal = voltage_a;
      m_calibration_data.hall_a_maximal = voltage_a;
    }

    if (m_calibration_data.hall_b_minimal == 0 && m_calibration_data.hall_b_maximal == 0) {
      m_calibration_data.hall_b_minimal = voltage_b;
      m_calibration_data.hall_b_maximal = voltage_b;
    }

    m_calibration_data.hall_a_minimal = std::min(m_calibration_data.hall_a_minimal, voltage_a);
    m_calibration_data.hall_a_maximal = std::max(m_calibration_data.hall_a_maximal, voltage_a);
    m_calibration_data.hall_b_minimal = std::min(m_calibration_data.hall_b_minimal, voltage_b);
    m_calibration_data.hall_b_maximal = std::max(m_calibration_data.hall_b_maximal, voltage_b);

    m_calibration_data.hall_a_minimal = std::clamp(m_calibration_data.hall_a_minimal, voltage_minimal, voltage_maximal);
    m_calibration_data.hall_a_maximal = std::clamp(m_calibration_data.hall_a_maximal, voltage_minimal, voltage_maximal);
    m_calibration_data.hall_b_minimal = std::clamp(m_calibration_data.hall_b_minimal, voltage_minimal, voltage_maximal);
    m_calibration_data.hall_b_maximal = std::clamp(m_calibration_data.hall_b_maximal, voltage_minimal, voltage_maximal);

    calibration_data = m_calibration_data;

    return type::SystemError::None;
  }

  auto set_minimal_position(type::Position const minimal_position) noexcept -> void { m_current_min_position = minimal_position; }

  auto set_maximal_position(type::Position const maximal_position) noexcept -> void { m_current_max_position = maximal_position; }

  auto get_position(type::Position& current_position) const noexcept -> type::SystemError {
    type::MilliVolt voltage_a = 0, voltage_b = 0;

    if (!m_driver_channel_a.get_voltage(voltage_a))
      return type::SystemError::AcceleratorReadFault;

    if (!m_driver_channel_b.get_voltage(voltage_b))
      return type::SystemError::AcceleratorReadFault;

    type::Position const pos_a =
        commons::map_range(voltage_a, m_calibration_data.hall_a_minimal, m_calibration_data.hall_a_maximal, m_current_min_position, m_current_max_position);
    type::Position const pos_b =
        commons::map_range(voltage_b, m_calibration_data.hall_b_minimal, m_calibration_data.hall_b_maximal, m_current_min_position, m_current_max_position);

    if (pos_a - pos_b > threshold) [[unlikely]] {
      return type::SystemError::AcceleratorMismatch;
    }

    current_position = pos_a;

    return type::SystemError::None;
  }
};

}  // namespace device
