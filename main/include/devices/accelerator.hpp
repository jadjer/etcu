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

#include <esp_adc/adc_oneshot.h>
#include <utility>
#include "types.hpp"

namespace devices {

using ADCHandle = adc_oneshot_unit_handle_t;

constexpr auto mapRange(MilliVolt const value, MilliVolt const fromMin, MilliVolt const fromMax) -> Position {
  if (fromMax == fromMin)
    return 0;

  return (value - fromMin) * 100 / (fromMax - fromMin);
}

template <ADCUnit Unit, ADCChannel ChannelA, ADCChannel ChannelB, MilliVolt Threshold>
class Accelerator {
  ADCHandle m_adc_handle{nullptr};

  MilliVolt m_calibrated_hall_a_minimal{0};
  MilliVolt m_calibrated_hall_a_maximal{0};
  MilliVolt m_calibrated_hall_b_minimal{0};
  MilliVolt m_calibrated_hall_b_maximal{0};

  Position m_current_min_position{0};
  Position m_current_max_position{100};

 public:
  void init() noexcept {
    adc_oneshot_unit_init_cfg_t constexpr handle_config = {
        .unit_id = Unit,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&handle_config, &m_adc_handle));

    adc_oneshot_chan_cfg_t constexpr channel_config = {
        .atten = ADC_ATTEN_DB_2_5,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(m_adc_handle, ChannelA, &channel_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(m_adc_handle, ChannelB, &channel_config));
  }

  auto calibrate(SystemError& current_errors) noexcept -> void {
    if (m_adc_handle == nullptr) {
      current_errors = current_errors | SystemError::AcceleratorInitFault;
      return;
    }

    int hall_a_raw = 0, hall_b_raw = 0;

    esp_err_t const errA = adc_oneshot_read(m_adc_handle, ChannelA, &hall_a_raw);
    esp_err_t const errB = adc_oneshot_read(m_adc_handle, ChannelB, &hall_b_raw);

    if (errA != ESP_OK or errB != ESP_OK) [[unlikely]] {
      current_errors = current_errors | SystemError::AcceleratorReadFault | SystemError::AcceleratorCalibrateFault;
      return;
    }

    auto const hall_a = static_cast<MilliVolt>(hall_a_raw);
    auto const hall_b = static_cast<MilliVolt>(hall_b_raw);

    m_calibrated_hall_a_minimal = std::min(m_calibrated_hall_a_minimal, hall_a);
    m_calibrated_hall_a_maximal = std::max(m_calibrated_hall_a_maximal, hall_a);

    m_calibrated_hall_b_minimal = std::min(m_calibrated_hall_b_minimal, hall_b);
    m_calibrated_hall_b_maximal = std::min(m_calibrated_hall_b_maximal, hall_b);
  }

  auto set_minimal_position(Position const minimal_position) noexcept -> void { m_current_min_position = minimal_position; }

  auto set_maximal_position(Position const maximal_position) noexcept -> void { m_current_max_position = maximal_position; }

  auto get_position(SystemError& current_errors) const noexcept -> Position {
    if (m_adc_handle == nullptr) {
      current_errors = current_errors | SystemError::AcceleratorInitFault;
      return 0;
    }

    if (m_calibrated_hall_a_maximal <= m_calibrated_hall_a_minimal or m_calibrated_hall_b_maximal <= m_calibrated_hall_b_minimal) [[unlikely]] {
      current_errors = current_errors | SystemError::AcceleratorCalibrateFault;
      return 0;
    }

    int hall_a_raw = 0, hall_b_raw = 0;

    esp_err_t const errA = adc_oneshot_read(m_adc_handle, ChannelA, &hall_a_raw);
    esp_err_t const errB = adc_oneshot_read(m_adc_handle, ChannelB, &hall_b_raw);

    if (errA != ESP_OK or errB != ESP_OK) [[unlikely]] {
      current_errors = current_errors | SystemError::AcceleratorReadFault;
      return 0;
    }

    MilliVolt const hall_a = std::clamp(static_cast<MilliVolt>(hall_a_raw), m_calibrated_hall_a_minimal, m_calibrated_hall_a_maximal);
    MilliVolt const hall_b = std::clamp(static_cast<MilliVolt>(hall_b_raw), m_calibrated_hall_b_minimal, m_calibrated_hall_b_maximal);

    auto const [minHall, maxHall] = std::minmax(hall_a, hall_b);

    MilliVolt const hall_diff = maxHall - minHall;

    if (std::cmp_greater(hall_diff, Threshold)) [[unlikely]] {
      current_errors = current_errors | SystemError::AcceleratorMismatch;
      return 0;
    }

    Position const currentPosition = mapRange(hall_a, m_calibrated_hall_a_minimal, m_calibrated_hall_a_maximal);

    return std::clamp(currentPosition, m_current_min_position, m_current_max_position);
  }
};

}  // namespace devices
