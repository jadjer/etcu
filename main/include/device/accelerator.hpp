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

#include "common/map_range.hpp"
#include "config/concepts.hpp"
#include "type/calibration.hpp"
#include "type/error.hpp"
#include "type/type.hpp"

namespace device {

template <class Driver, std::uint8_t HallA, std::uint8_t HallB, type::Position Threshold>
  requires concepts::ADC<Driver>
class Accelerator {
  static constexpr std::uint8_t hall_a{HallA};
  static constexpr std::uint8_t hall_b{HallB};

  Driver& m_driver_adc;

  type::AcceleratorCalibrationData m_calibration_data{
      .hall_a_minimal{650},
      .hall_a_maximal{1350},
      .hall_b_minimal{320},
      .hall_b_maximal{690},
  };

 public:
  constexpr explicit Accelerator(Driver& driver_adc) noexcept : m_driver_adc(driver_adc) {}

  constexpr Accelerator() noexcept = delete;

  Accelerator(Accelerator const&) noexcept = delete;
  auto operator=(Accelerator const&) noexcept -> Accelerator& = delete;

  Accelerator(Accelerator&&) noexcept = delete;
  auto operator=(Accelerator&&) noexcept -> Accelerator& = delete;

  constexpr ~Accelerator() noexcept = default;

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_driver_adc.init()) [[unlikely]] {
      return type::SystemError::AcceleratorInitFault;
    }

    if (!m_driver_adc.template configure_channel<hall_a>()) [[unlikely]] {
      return type::SystemError::AcceleratorInitFault;
    }

    if (!m_driver_adc.template configure_channel<hall_b>()) [[unlikely]] {
      return type::SystemError::AcceleratorInitFault;
    }

    return type::SystemError::None;
  }

  auto set_calibration(type::AcceleratorCalibrationData const& calibration_data) noexcept -> void { m_calibration_data = calibration_data; }

  [[nodiscard]] auto get_position(type::Position& current_position) noexcept -> type::SystemError {
    static constexpr type::Position value_min{type::Position::value_min};
    static constexpr type::Position value_max{type::Position::value_max};
    static constexpr type::Position threshold{Threshold};

    type::AccPosition adc_value_a, adc_value_b;

    if (!m_driver_adc.template get_value<hall_a>(adc_value_a)) [[unlikely]] {
      return type::SystemError::AcceleratorReadFault;
    }

    if (!m_driver_adc.template get_value<hall_b>(adc_value_b)) [[unlikely]] {
      return type::SystemError::AcceleratorReadFault;
    }

    type::Position const pos_a = common::map_range(adc_value_a, m_calibration_data.hall_a_minimal, m_calibration_data.hall_a_maximal, value_min, value_max);
    type::Position const pos_b = common::map_range(adc_value_b, m_calibration_data.hall_b_minimal, m_calibration_data.hall_b_maximal, value_min, value_max);

    if (type::Position const raw_diff{std::abs(pos_a.value - pos_b.value)}; raw_diff > threshold) [[unlikely]] {
      current_position = type::Position{0};
      return type::SystemError::AcceleratorMismatch;
    }

    current_position = pos_a;

    return type::SystemError::None;
  }
};

}  // namespace device
