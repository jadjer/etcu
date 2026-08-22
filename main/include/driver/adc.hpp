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
// Created by jadjer on 18.08.26.
//

#pragma once

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>

namespace driver {

using ADCUnitId = adc_unit_t;
using ADCHandle = adc_oneshot_unit_handle_t;
using ADCChannelId = adc_channel_t;
using ADCAttenuation = adc_atten_t;
using ADCCalibrationHandle = adc_cali_handle_t;
using ADCCalibrationHandles = std::array<ADCCalibrationHandle, 11>;
using ADCHandleConfig = adc_oneshot_unit_init_cfg_t;
using ADCChannelConfig = adc_oneshot_chan_cfg_t;
using ADCCalibrationConfig = adc_cali_curve_fitting_config_t;

template <ADCUnitId unitId, ADCAttenuation attenuation = ADC_ATTEN_DB_6>
class ADC {
  ADCHandle m_handle{nullptr};
  ADCCalibrationHandles m_calibration_handles{};

 public:
  constexpr ADC() noexcept = default;

  [[nodiscard]] auto init() noexcept -> bool {
    if (m_handle != nullptr) [[unlikely]]
      return true;

    ADCHandleConfig constexpr handle_config = {
        .unit_id = unitId,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    return adc_oneshot_new_unit(&handle_config, &m_handle) == ESP_OK;
  }

  template <ADCChannelId channelId>
  [[nodiscard]] auto configure_channel() noexcept -> bool {
    if (m_handle == nullptr) [[unlikely]]
      return false;

    ADCChannelConfig constexpr channel_config = {
        .atten = attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(m_handle, channelId, &channel_config) != ESP_OK)
      return false;

    ADCCalibrationConfig constexpr calibration_config = {
        .unit_id = unitId,
        .chan = channelId,
        .atten = attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    auto const channel_index = static_cast<type::Size>(channelId);
    if (adc_cali_create_scheme_curve_fitting(&calibration_config, &m_calibration_handles[channel_index]) != ESP_OK)
      return false;

    return true;
  }

  template <ADCChannelId channelId, typename T>
  [[nodiscard]] auto get_voltage(T& value) const noexcept -> bool {
    if (m_handle == nullptr) [[unlikely]]
      return false;

    auto const channel_index = static_cast<type::Size>(channelId);
    auto const calibration_handle = m_calibration_handles[channel_index];
    if (calibration_handle == nullptr) [[unlikely]]
      return false;

    int raw_value = 0;

    if (adc_oneshot_read(m_handle, channelId, &raw_value) != ESP_OK) [[unlikely]]
      return false;

    int voltage = 0;

    if (adc_cali_raw_to_voltage(calibration_handle, raw_value, &voltage) != ESP_OK) [[unlikely]] {
      return false;
    }

    value = static_cast<T>(voltage);

    return true;
  }
};

}  // namespace driver
