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
      .hall_a_minimal = type::MilliVolt{620},
      .hall_a_maximal = type::MilliVolt{1320},
      .hall_b_minimal = type::MilliVolt{220},
      .hall_b_maximal = type::MilliVolt{460},
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
    type::MilliVolt voltage_a{0};
    type::MilliVolt voltage_b{0};

    if (!m_driver_channel_a.get_voltage(voltage_a))
      return type::SystemError::AcceleratorReadFault;

    if (!m_driver_channel_b.get_voltage(voltage_b))
      return type::SystemError::AcceleratorReadFault;

    int64_t const v_a = voltage_a.get();
    int64_t const v_b = voltage_b.get();

    int64_t const a_min = m_calibration_data.hall_a_minimal.get();
    int64_t const a_max = m_calibration_data.hall_a_maximal.get();
    int64_t const b_min = m_calibration_data.hall_b_minimal.get();
    int64_t const b_max = m_calibration_data.hall_b_maximal.get();

    m_calibration_data.hall_a_minimal = type::MilliVolt{std::min(a_min, v_a)};
    m_calibration_data.hall_a_maximal = type::MilliVolt{std::max(a_max, v_a)};
    m_calibration_data.hall_b_minimal = type::MilliVolt{std::min(b_min, v_b)};
    m_calibration_data.hall_b_maximal = type::MilliVolt{std::max(b_max, v_b)};

    calibration_data = m_calibration_data;

    return type::SystemError::None;
  }

  auto set_minimal_position(type::Position const minimal_position) noexcept -> void { m_current_min_position = minimal_position; }

  auto set_maximal_position(type::Position const maximal_position) noexcept -> void { m_current_max_position = maximal_position; }

  auto get_position(type::Position& current_position) const noexcept -> type::SystemError {
    type::MilliVolt voltage_a{0};
    type::MilliVolt voltage_b{0};

    if (!m_driver_channel_a.get_voltage(voltage_a))
      return type::SystemError::AcceleratorReadFault;

    if (!m_driver_channel_b.get_voltage(voltage_b))
      return type::SystemError::AcceleratorReadFault;

    // Масштабируем вольтаж в проценты положения педали [0..100%]
    type::Position const pos_a =
        common::map_range(voltage_a, m_calibration_data.hall_a_minimal, m_calibration_data.hall_a_maximal, m_current_min_position, m_current_max_position);
    type::Position const pos_b =
        common::map_range(voltage_b, m_calibration_data.hall_b_minimal, m_calibration_data.hall_b_maximal, m_current_min_position, m_current_max_position);

    // ИСПРАВЛЕНИЕ: Вычисляем разность по модулю в чистом int64_t.
    // Это полностью защищает от усечения отрицательных чисел в clamp-конструкторе.
    int64_t const raw_diff = std::abs(static_cast<int64_t>(pos_a.get()) - static_cast<int64_t>(pos_b.get()));

    // Сравниваем сырую дельту int64_t с порогом threshold (у которого операторы сравнения с числами перегружены)
    if (raw_diff > threshold) [[unlikely]] {
      current_position = type::Position{0};  // Явное конструирование explicit типа

      return type::SystemError::AcceleratorMismatch;
    }

    current_position = pos_a;

    return type::SystemError::None;
  }
};

}  // namespace device
