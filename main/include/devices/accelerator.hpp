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

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_oneshot.h>
#include <utility>
#include "types.hpp"

namespace devices {

using ADCHandle = adc_oneshot_unit_handle_t;
using ADCCalibrationHandle = adc_cali_handle_t;

constexpr auto mapRange(MilliVolt const value, MilliVolt const fromMin, MilliVolt const fromMax) -> Position {
  if (fromMax == fromMin)
    return 0;

  return (value - fromMin) * 100 / (fromMax - fromMin);
}

template <ADCUnit Unit, ADCChannel ChannelA, ADCChannel ChannelB, MilliVolt Threshold, adc_atten_t Attenuation = ADC_ATTEN_DB_2_5>
class Accelerator {
  static MilliVolt constexpr MinVoltage = 0;
  static MilliVolt constexpr MaxVoltage = 3100;
  static Position constexpr MinPosition = 0;
  static Position constexpr MaxPosition = 100;
  static MilliVolt constexpr AccelZeroOffsetMv = 100;

  ADCHandle m_handle{nullptr};
  ADCCalibrationHandle m_calibration_channel_a_handle{nullptr};
  ADCCalibrationHandle m_calibration_channel_b_handle{nullptr};

  MilliVolt m_calibrated_hall_a_minimal{MinVoltage};
  MilliVolt m_calibrated_hall_a_maximal{MinVoltage};
  MilliVolt m_calibrated_hall_b_minimal{MinVoltage};
  MilliVolt m_calibrated_hall_b_maximal{MinVoltage};

  Position m_current_min_position{MinPosition};
  Position m_current_max_position{MaxPosition};

 public:
  auto init() noexcept -> SystemError {
    adc_oneshot_unit_init_cfg_t constexpr handle_config = {
        .unit_id = Unit,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (auto const err = adc_oneshot_new_unit(&handle_config, &m_handle); err != ESP_OK) {
      return SystemError::AcceleratorInitFault;
    }

    adc_oneshot_chan_cfg_t constexpr channel_config = {
        .atten = Attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (auto const err = adc_oneshot_config_channel(m_handle, ChannelA, &channel_config); err != ESP_OK) {
      return SystemError::AcceleratorInitFault;
    }
    if (auto const err = adc_oneshot_config_channel(m_handle, ChannelB, &channel_config); err != ESP_OK) {
      return SystemError::AcceleratorInitFault;
    }

    adc_cali_curve_fitting_config_t const calibration_a_config = {
        .unit_id = Unit,
        .chan = ChannelA,
        .atten = Attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (auto const err = adc_cali_create_scheme_curve_fitting(&calibration_a_config, &m_calibration_channel_a_handle); err != ESP_OK) {
      return SystemError::AcceleratorInitFault;
    }

    adc_cali_curve_fitting_config_t const calibration_b_config = {
        .unit_id = Unit,
        .chan = ChannelB,
        .atten = Attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (auto const err = adc_cali_create_scheme_curve_fitting(&calibration_b_config, &m_calibration_channel_b_handle); err != ESP_OK) {
      return SystemError::AcceleratorInitFault;
    }

    return SystemError::None;
  }

  auto calibrate() noexcept -> SystemError {
    if (m_handle == nullptr) {
      return SystemError::AcceleratorInitFault;
    }

    int voltage_a = 0, voltage_b = 0;

    {
      int hall_a_raw = 0, hall_b_raw = 0;

      esp_err_t const err_raw_a = adc_oneshot_read(m_handle, ChannelA, &hall_a_raw);
      esp_err_t const err_raw_b = adc_oneshot_read(m_handle, ChannelB, &hall_b_raw);

      if (err_raw_a != ESP_OK or err_raw_b != ESP_OK) [[unlikely]] {
        return SystemError::AcceleratorReadFault | SystemError::AcceleratorCalibrateFault;
      }

      esp_err_t const err_voltage_a = adc_cali_raw_to_voltage(m_calibration_channel_a_handle, hall_a_raw, &voltage_a);
      esp_err_t const err_voltage_b = adc_cali_raw_to_voltage(m_calibration_channel_b_handle, hall_b_raw, &voltage_b);

      if (err_voltage_a != ESP_OK or err_voltage_b != ESP_OK) [[unlikely]] {
        return SystemError::AcceleratorReadFault | SystemError::AcceleratorCalibrateFault;
      }
    }

    auto const hall_a = static_cast<MilliVolt>(voltage_a);
    auto const hall_b = static_cast<MilliVolt>(voltage_b);

    m_calibrated_hall_a_minimal = std::clamp(hall_a + AccelZeroOffsetMv, 0, 3300);
    m_calibrated_hall_b_minimal = std::clamp(hall_b + AccelZeroOffsetMv, 0, 3300);

    // 3. Фиксируем максимумы (полный газ)
    m_calibrated_hall_a_maximal = std::max(m_calibrated_hall_a_maximal, hall_a);
    m_calibrated_hall_b_maximal = std::max(m_calibrated_hall_b_maximal, hall_b);

    ESP_LOGI("ACC", "%d %d %d %d %d %d", hall_a, hall_b, m_calibrated_hall_a_minimal, m_calibrated_hall_a_maximal, m_calibrated_hall_b_minimal,
             m_calibrated_hall_b_maximal);

    return SystemError::None;
  }

  auto set_minimal_position(Position const minimal_position) noexcept -> void { m_current_min_position = minimal_position; }

  auto set_maximal_position(Position const maximal_position) noexcept -> void { m_current_max_position = maximal_position; }

  auto get_position(Position& position) const noexcept -> SystemError {
    if (m_handle == nullptr) {
      return SystemError::AcceleratorInitFault;
    }

    int hall_a_raw = 0, hall_b_raw = 0;

    esp_err_t const err_a = adc_oneshot_read(m_handle, ChannelA, &hall_a_raw);
    esp_err_t const err_b = adc_oneshot_read(m_handle, ChannelB, &hall_b_raw);

    if (err_a != ESP_OK or err_b != ESP_OK) [[unlikely]] {
      return SystemError::AcceleratorReadFault;
    }

    int voltage_a = 0, voltage_b = 0;

    esp_err_t const err_voltage_a = adc_cali_raw_to_voltage(m_calibration_channel_a_handle, hall_a_raw, &voltage_a);
    esp_err_t const err_voltage_b = adc_cali_raw_to_voltage(m_calibration_channel_b_handle, hall_b_raw, &voltage_b);

    if (err_voltage_a != ESP_OK or err_voltage_b != ESP_OK) [[unlikely]] {
      return SystemError::AcceleratorReadFault | SystemError::AcceleratorCalibrateFault;
    }

    MilliVolt const hall_a = std::clamp(static_cast<MilliVolt>(voltage_a), m_calibrated_hall_a_minimal, m_calibrated_hall_a_maximal);
    MilliVolt const hall_b = std::clamp(static_cast<MilliVolt>(voltage_b), m_calibrated_hall_b_minimal, m_calibrated_hall_b_maximal);

    // MilliVolt const hall_diff = maxHall - minHall;

    ESP_LOGI("ACC", "A: %d, B: %d", hall_a, hall_b);

    // if (std::cmp_greater(hall_diff, Threshold)) [[unlikely]] {
    //   current_errors = current_errors | SystemError::AcceleratorMismatch;
    //   return 0;
    // }

    Position const currentPosition = mapRange(hall_a, m_calibrated_hall_a_minimal, m_calibrated_hall_a_maximal);

    position = std::clamp(currentPosition, m_current_min_position, m_current_max_position);

    return SystemError::None;
  }
};

}  // namespace devices
