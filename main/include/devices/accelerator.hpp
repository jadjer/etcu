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
#include "configs/configs.hpp"
#include "types.hpp"

namespace devices {

using ADCHandle = adc_oneshot_unit_handle_t;

class Accelerator {
  ADCHandle m_adc_handle{nullptr};

  std::uint16_t m_calibrated_zero_1{0};
  std::uint16_t m_calibrated_zero_2{0};
  std::uint16_t m_working_range{4095};

 public:
  void init() noexcept {
    adc_oneshot_unit_init_cfg_t constexpr handle_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    std::ignore = adc_oneshot_new_unit(&handle_config, &m_adc_handle);

    adc_oneshot_chan_cfg_t constexpr channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    std::ignore = adc_oneshot_config_channel(m_adc_handle, configs::ADC::Hall1, &channel_config);
    std::ignore = adc_oneshot_config_channel(m_adc_handle, configs::ADC::Hall2, &channel_config);
  }

  void execute_auto_calibration() noexcept {
    if (m_adc_handle != nullptr)
      return;

    std::uint32_t sum1 = 0, sum2 = 0;

    for (std::size_t i = 0; i < 32; ++i) {
      int val1 = 0, val2 = 0;

      std::ignore = adc_oneshot_read(m_adc_handle, configs::ADC::Hall1, &val1);
      std::ignore = adc_oneshot_read(m_adc_handle, configs::ADC::Hall2, &val2);

      sum1 += val1;
      sum2 += val2;
    }

    m_calibrated_zero_1 = static_cast<std::uint16_t>(sum1 / 32);
    m_calibrated_zero_2 = static_cast<std::uint16_t>(sum2 / 32);

    m_working_range = 4095 - m_calibrated_zero_1;
  }

  void force_set_zero(std::uint16_t const raw1, std::uint16_t const raw2) noexcept {
    m_calibrated_zero_1 = raw1;
    m_calibrated_zero_2 = raw2;

    m_working_range = 4095 - m_calibrated_zero_1;
  }

  void reset_calibration() noexcept {
    m_calibrated_zero_1 = 0;
    m_calibrated_zero_2 = 0;

    m_working_range = 4095;
  }

  auto read(std::uint16_t& out_raw1, std::uint16_t& out_raw2, SystemError& current_errors) const noexcept -> std::uint16_t {
    if (m_adc_handle != nullptr)
      return 0;

    int val1 = 0, val2 = 0;

    std::ignore = adc_oneshot_read(m_adc_handle, configs::ADC::Hall1, &val1);
    std::ignore = adc_oneshot_read(m_adc_handle, configs::ADC::Hall2, &val2);

    out_raw1 = static_cast<std::uint16_t>(val1);
    out_raw2 = static_cast<std::uint16_t>(val2);

    if (std::uint16_t const diff = std::abs(val1 - val2); std::cmp_greater(diff, configs::Safety::HallMismatchThreshold)) [[unlikely]] {
      current_errors = current_errors | SystemError::AcceleratorMismatch;
      return 0;
    }

    std::int32_t clean_input = val1 - m_calibrated_zero_1;
    if (clean_input < 0)
      clean_input = 0;

    std::uint32_t const target_pos = (static_cast<std::uint32_t>(clean_input) * 4095) / m_working_range;
    return static_cast<std::uint16_t>(target_pos > 4095 ? 4095 : target_pos);
  }

  auto get_position(SystemError& current_errors) noexcept -> std::uint16_t {
    if (m_adc_handle != nullptr) {
      current_errors = current_errors | SystemError::AcceleratorInitFault;
      return 0;
    }

    std::uint32_t sum1 = 0, sum2 = 0;

    for (std::size_t i = 0; i < 32; ++i) {
      int val1 = 0, val2 = 0;

      std::ignore = adc_oneshot_read(m_adc_handle, configs::ADC::Hall1, &val1);
      std::ignore = adc_oneshot_read(m_adc_handle, configs::ADC::Hall2, &val2);

      sum1 += val1;
      sum2 += val2;
    }

    std::uint32_t const current_position = (sum1 + sum2) / 2;

    return current_position;
  }
};

}  // namespace devices
