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
#include <array>
#include <cstddef>
#include <tuple>

namespace driver {

enum class ADCUnit : std::uint8_t {
  Unit1 = 0,
  Unit2 = 1,
};
static_assert(static_cast<int>(ADCUnit::Unit1) == static_cast<int>(ADC_UNIT_1), "CRITICAL: driver::ADCUnit::Unit1 value mismatch with ESP-IDF ADC_UNIT_1!");
static_assert(static_cast<int>(ADCUnit::Unit2) == static_cast<int>(ADC_UNIT_2), "CRITICAL: driver::ADCUnit::Unit2 value mismatch with ESP-IDF ADC_UNIT_2!");

enum class ADCAttenuation : std::uint8_t {
  Db0 = ADC_ATTEN_DB_0,
  Db2_5 = ADC_ATTEN_DB_2_5,
  Db6 = ADC_ATTEN_DB_6,
  Db12 = ADC_ATTEN_DB_12,
};
static_assert(static_cast<int>(ADCAttenuation::Db0) == static_cast<int>(ADC_ATTEN_DB_0),
              "CRITICAL: driver::ADCAttenuation::Db0 value mismatch with ESP-IDF ADC_ATTEN_DB_0!");
static_assert(static_cast<int>(ADCAttenuation::Db2_5) == static_cast<int>(ADC_ATTEN_DB_2_5),
              "CRITICAL: driver::ADCAttenuation::Db2_5 value mismatch with ESP-IDF ADC_ATTEN_DB_2_5!");
static_assert(static_cast<int>(ADCAttenuation::Db6) == static_cast<int>(ADC_ATTEN_DB_6),
              "CRITICAL: driver::ADCAttenuation::Db6 value mismatch with ESP-IDF ADC_ATTEN_DB_6!");
static_assert(static_cast<int>(ADCAttenuation::Db12) == static_cast<int>(ADC_ATTEN_DB_12),
              "CRITICAL: driver::ADCAttenuation::Db12 value mismatch with ESP-IDF ADC_ATTEN_DB_12!");

template <ADCUnit UnitId, ADCAttenuation Attenuation = ADCAttenuation::Db12>
class ADC {
  static constexpr std::uint8_t calibration_size{11};
  static constexpr auto esp_unit{static_cast<adc_unit_t>(UnitId)};
  static constexpr auto esp_attenuation{static_cast<adc_atten_t>(Attenuation)};

  adc_oneshot_unit_handle_t m_handle{nullptr};
  std::array<adc_cali_handle_t, calibration_size> m_calibration_handles{};

 public:
  constexpr ADC() noexcept = default;

  ADC(ADC const&) noexcept = delete;
  auto operator=(ADC const&) noexcept -> ADC& = delete;

  ~ADC() noexcept {
    for (auto const handle : m_calibration_handles) {
      if (handle != nullptr)
        std::ignore = adc_cali_delete_scheme_curve_fitting(handle);
    }

    if (m_handle != nullptr)
      std::ignore = adc_oneshot_del_unit(m_handle);
  }

  [[nodiscard]] auto init() noexcept -> bool {
    if (m_handle != nullptr) [[unlikely]]
      return true;

    adc_oneshot_unit_init_cfg_t const handle_config = {
        .unit_id = esp_unit,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    return adc_oneshot_new_unit(&handle_config, &m_handle) == ESP_OK;
  }

  template <int ChannelId>
  [[nodiscard]] auto configure_channel() noexcept -> bool {
    if (m_handle == nullptr && !init()) [[unlikely]]
      return false;

    constexpr auto esp_channel = static_cast<adc_channel_t>(ChannelId);

    adc_oneshot_chan_cfg_t const channel_config = {
        .atten = esp_attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    if (esp_err_t const error = adc_oneshot_config_channel(m_handle, esp_channel, &channel_config); error != ESP_OK) [[unlikely]]
      return false;

    adc_cali_curve_fitting_config_t const calibration_config = {
        .unit_id = esp_unit,
        .chan = esp_channel,
        .atten = esp_attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    constexpr auto channel_index = static_cast<std::size_t>(ChannelId);

    if (m_calibration_handles[channel_index] == nullptr) {
      if (esp_err_t const error = adc_cali_create_scheme_curve_fitting(&calibration_config, &m_calibration_handles[channel_index]); error != ESP_OK)
          [[unlikely]]
        return false;
    }

    return true;
  }

  template <int ChannelId, typename T>
  [[nodiscard]] auto get_voltage(T& value) const noexcept -> bool {
    if (m_handle == nullptr) [[unlikely]]
      return false;

    constexpr auto channel_index = static_cast<std::size_t>(ChannelId);

    auto const calibration_handle = m_calibration_handles[channel_index];
    if (calibration_handle == nullptr) [[unlikely]]
      return false;

    constexpr auto esp_channel = static_cast<adc_channel_t>(ChannelId);

    int raw_value = 0;
    if (esp_err_t const error = adc_oneshot_read(m_handle, esp_channel, &raw_value); error != ESP_OK) [[unlikely]]
      return false;

    int voltage = 0;
    if (esp_err_t const error = adc_cali_raw_to_voltage(calibration_handle, raw_value, &voltage); error != ESP_OK) [[unlikely]]
      return false;

    value = T{static_cast<std::uint16_t>(voltage)};

    return true;
  }
};

}  // namespace driver
