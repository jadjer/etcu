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

#include <driver/gpio.h>

namespace driver {

enum class GPIOConfigMode : std::uint8_t {
  Disable = 0,
  Input = 1,
  Output = 2,
  InputOutput = 3,
  OutputOpenDrain = 6,
  InputOutputOpedDrain = 7,
};

static_assert(static_cast<int>(GPIOConfigMode::Disable) == static_cast<int>(GPIO_MODE_DISABLE));
static_assert(static_cast<int>(GPIOConfigMode::Input) == static_cast<int>(GPIO_MODE_INPUT));
static_assert(static_cast<int>(GPIOConfigMode::Output) == static_cast<int>(GPIO_MODE_OUTPUT));
static_assert(static_cast<int>(GPIOConfigMode::InputOutput) == static_cast<int>(GPIO_MODE_INPUT_OUTPUT));
static_assert(static_cast<int>(GPIOConfigMode::OutputOpenDrain) == static_cast<int>(GPIO_MODE_OUTPUT_OD));
static_assert(static_cast<int>(GPIOConfigMode::InputOutputOpedDrain) == static_cast<int>(GPIO_MODE_INPUT_OUTPUT_OD));

template <std::uint8_t Pin, GPIOConfigMode Mode, bool Inverse = false>
  requires(Pin < GPIO_NUM_MAX)
class GPIO {
  static constexpr bool inverse{Inverse};
  static constexpr auto esp_pin{static_cast<gpio_num_t>(Pin)};

  static constexpr auto is_input() noexcept -> bool {
    return Mode == GPIOConfigMode::Input || Mode == GPIOConfigMode::InputOutput || Mode == GPIOConfigMode::InputOutputOpedDrain;
  }

  static constexpr bool need_pull_up = is_input() && inverse;
  static constexpr bool need_pull_down = is_input() && !inverse;

  static constexpr gpio_mode_t get_esp_mode() noexcept {
    switch (Mode) {
      case GPIOConfigMode::Input:
        return GPIO_MODE_INPUT;
      case GPIOConfigMode::Output:
        return GPIO_MODE_OUTPUT;
      case GPIOConfigMode::InputOutput:
        return GPIO_MODE_INPUT_OUTPUT;
      case GPIOConfigMode::OutputOpenDrain:
        return GPIO_MODE_OUTPUT_OD;
      case GPIOConfigMode::InputOutputOpedDrain:
        return GPIO_MODE_INPUT_OUTPUT_OD;
      default:
        return GPIO_MODE_DISABLE;
    }
  }

 public:
  constexpr GPIO() noexcept = default;

  GPIO(GPIO const&) noexcept = delete;
  auto operator=(GPIO const&) noexcept -> GPIO& = delete;

  GPIO(GPIO&&) noexcept = delete;
  auto operator=(GPIO&&) noexcept -> GPIO& = delete;

  constexpr ~GPIO() noexcept = default;

  static auto init() noexcept -> bool {
    static constexpr gpio_config_t config{
        .pin_bit_mask = 1ULL << esp_pin,
        .mode = get_esp_mode(),
        .pull_up_en = need_pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = need_pull_down ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (esp_err_t const error = gpio_reset_pin(esp_pin); error != ESP_OK) [[unlikely]] {
      ESP_LOGE("GPIO", "gpio_reset_pin %s", esp_err_to_name(error));
      return false;
    }

    if (esp_err_t const error = gpio_config(&config); error != ESP_OK) [[unlikely]] {
      ESP_LOGE("GPIO", "gpio_config %s", esp_err_to_name(error));
      return false;
    }

    return true;
  }

  [[nodiscard]] static auto get_level() noexcept -> bool {
    int const level = gpio_get_level(esp_pin);
    return inverse ? level == 0 : level == 1;
  }

  static auto set_level(bool const level) noexcept -> bool {
    if (esp_err_t const error = gpio_set_level(esp_pin, level ^ inverse); error != ESP_OK) [[unlikely]] {
      ESP_LOGE("GPIO", "gpio_set_level %s", esp_err_to_name(error));
      return false;
    }

    return true;
  }

  static auto enable() noexcept -> bool { return set_level(true); }

  static auto disable() noexcept -> bool { return set_level(false); }
};

}  // namespace driver
