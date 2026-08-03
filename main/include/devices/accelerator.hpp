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
#include <algorithm>
#include "commons/map_range.hpp"
#include "types.hpp"

namespace devices {

using ADCHandle = adc_oneshot_unit_handle_t;
using ADCCalibrationHandle = adc_cali_handle_t;

template <ADCUnit unit, ADCChannel channelA, ADCChannel channelB, Position threshold, adc_atten_t attenuation = ADC_ATTEN_DB_12>
class Accelerator {
  ADCHandle m_handle{nullptr};
  ADCCalibrationHandle m_calibration_channel_a_handle{nullptr};
  ADCCalibrationHandle m_calibration_channel_b_handle{nullptr};

  MilliVolt m_calibrated_hall_a_minimal{620};
  MilliVolt m_calibrated_hall_b_minimal{220};

  MilliVolt m_calibrated_hall_a_maximal{1300};
  MilliVolt m_calibrated_hall_b_maximal{450};

  Position m_current_min_position{0};
  Position m_current_max_position{100};

  static Position constexpr MaxChannelsMismatchPercent = 5;

  auto get_voltages(MilliVolt& hall_a, MilliVolt& hall_b) const -> SystemError {
    if (m_handle == nullptr) {
      return SystemError::AcceleratorInitFault;
    }

    int voltage_a = 0, voltage_b = 0;

    {
      int hall_a_raw = 0, hall_b_raw = 0;

      esp_err_t const err_raw_a = adc_oneshot_read(m_handle, channelA, &hall_a_raw);
      esp_err_t const err_raw_b = adc_oneshot_read(m_handle, channelB, &hall_b_raw);

      if (err_raw_a != ESP_OK or err_raw_b != ESP_OK) [[unlikely]] {
        return SystemError::AcceleratorReadFault;
      }

      esp_err_t const err_voltage_a = adc_cali_raw_to_voltage(m_calibration_channel_a_handle, hall_a_raw, &voltage_a);
      esp_err_t const err_voltage_b = adc_cali_raw_to_voltage(m_calibration_channel_b_handle, hall_b_raw, &voltage_b);

      if (err_voltage_a != ESP_OK or err_voltage_b != ESP_OK) [[unlikely]] {
        return SystemError::AcceleratorReadFault;
      }
    }

    hall_a = static_cast<MilliVolt>(voltage_a);
    hall_b = static_cast<MilliVolt>(voltage_b);

    return SystemError::None;
  }

 public:
  auto init() noexcept -> SystemError {
    adc_oneshot_unit_init_cfg_t constexpr handle_config = {
        .unit_id = unit,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (auto const err = adc_oneshot_new_unit(&handle_config, &m_handle); err != ESP_OK) {
      return SystemError::AcceleratorInitFault;
    }

    adc_oneshot_chan_cfg_t constexpr channel_config = {
        .atten = attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (auto const err = adc_oneshot_config_channel(m_handle, channelA, &channel_config); err != ESP_OK) {
      return SystemError::AcceleratorInitFault;
    }
    if (auto const err = adc_oneshot_config_channel(m_handle, channelB, &channel_config); err != ESP_OK) {
      return SystemError::AcceleratorInitFault;
    }

    adc_cali_curve_fitting_config_t const calibration_a_config = {
        .unit_id = unit,
        .chan = channelA,
        .atten = attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (auto const err = adc_cali_create_scheme_curve_fitting(&calibration_a_config, &m_calibration_channel_a_handle); err != ESP_OK) {
      return SystemError::AcceleratorInitFault;
    }

    adc_cali_curve_fitting_config_t const calibration_b_config = {
        .unit_id = unit,
        .chan = channelB,
        .atten = attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (auto const err = adc_cali_create_scheme_curve_fitting(&calibration_b_config, &m_calibration_channel_b_handle); err != ESP_OK) {
      return SystemError::AcceleratorInitFault;
    }

    return SystemError::None;
  }

  auto calibrate() noexcept -> SystemError {
    MilliVolt voltage_a = 0, voltage_b = 0;

    SystemError err = get_voltages(voltage_a, voltage_b);
    if (err != SystemError::None) {
      return err;
    }

    m_calibrated_hall_a_minimal = std::clamp(voltage_a + 50, 0, 3300);
    m_calibrated_hall_b_minimal = std::clamp(voltage_b + 50, 0, 3300);

    m_calibrated_hall_a_maximal = std::max(m_calibrated_hall_a_maximal, voltage_a);
    m_calibrated_hall_b_maximal = std::max(m_calibrated_hall_b_maximal, voltage_b);

    ESP_LOGI("ACC", "%d %d %d %d %d %d", voltage_a, voltage_b, m_calibrated_hall_a_minimal, m_calibrated_hall_a_maximal, m_calibrated_hall_b_minimal,
             m_calibrated_hall_b_maximal);

    return SystemError::None;
  }

  auto set_minimal_position(Position const minimal_position) noexcept -> void { m_current_min_position = minimal_position; }

  auto set_maximal_position(Position const maximal_position) noexcept -> void { m_current_max_position = maximal_position; }

  auto get_position(Position& current_position) const noexcept -> SystemError {
    MilliVolt voltage_a = 0, voltage_b = 0;

    SystemError err = get_voltages(voltage_a, voltage_b);
    if (err != SystemError::None) {
      return err;
    }

    Position const pos_a = commons::map_range(voltage_a, m_calibrated_hall_a_minimal, m_calibrated_hall_a_maximal, m_current_min_position, m_current_max_position);
    Position const pos_b = commons::map_range(voltage_b, m_calibrated_hall_b_minimal, m_calibrated_hall_b_maximal, m_current_min_position, m_current_max_position);

    if (std::abs(pos_a - pos_b) > threshold) [[unlikely]] {
      return SystemError::AcceleratorMismatch;
    }

    Position const deadband = std::abs(m_current_max_position - m_current_min_position) * 2 / 100;

    if (std::abs(pos_a - m_current_min_position) < deadband) {
      current_position = m_current_min_position;
    } else {
      current_position = pos_a;
    }

    return SystemError::None;
  }
};

}  // namespace devices
