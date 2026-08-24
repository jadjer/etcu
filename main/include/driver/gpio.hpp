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
static_assert(static_cast<int>(GPIOConfigMode::Disable) == static_cast<int>(GPIO_MODE_DISABLE),
              "CRITICAL: driver::GPIOConfigMode::Disable value mismatch with GPIO_MODE_DISABLE!");
static_assert(static_cast<int>(GPIOConfigMode::Input) == static_cast<int>(GPIO_MODE_INPUT),
              "CRITICAL: driver::GPIOConfigMode::Input value mismatch with GPIO_MODE_INPUT!");
static_assert(static_cast<int>(GPIOConfigMode::Output) == static_cast<int>(GPIO_MODE_OUTPUT),
              "CRITICAL: driver::GPIOConfigMode::Output value mismatch with GPIO_MODE_INPUT!");
static_assert(static_cast<int>(GPIOConfigMode::InputOutput) == static_cast<int>(GPIO_MODE_INPUT_OUTPUT),
              "CRITICAL: driver::GPIOConfigMode::InputOutput value mismatch with GPIO_MODE_INPUT_OUTPUT!");
static_assert(static_cast<int>(GPIOConfigMode::OutputOpenDrain) == static_cast<int>(GPIO_MODE_OUTPUT_OD),
              "CRITICAL: driver::GPIOConfigMode::OutputOpenDrain value mismatch with GPIO_MODE_OUTPUT_OD!");
static_assert(static_cast<int>(GPIOConfigMode::InputOutputOpedDrain) == static_cast<int>(GPIO_MODE_INPUT_OUTPUT_OD),
              "CRITICAL: driver::GPIOConfigMode::InputOutputOpedDrain value mismatch with GPIO_MODE_INPUT_OUTPUT_OD!");

template <int Pin, GPIOConfigMode Mode, bool Inverse = false>
struct GPIO {
  static constexpr auto esp_pin{static_cast<gpio_num_t>(Pin)};

  static constexpr gpio_mode_t get_esp_mode() noexcept {
    if constexpr (Mode == GPIOConfigMode::Input)
      return GPIO_MODE_INPUT;
    if constexpr (Mode == GPIOConfigMode::Output)
      return GPIO_MODE_OUTPUT;
    if constexpr (Mode == GPIOConfigMode::InputOutput)
      return GPIO_MODE_INPUT_OUTPUT;
    if constexpr (Mode == GPIOConfigMode::OutputOpenDrain)
      return GPIO_MODE_OUTPUT_OD;
    if constexpr (Mode == GPIOConfigMode::InputOutputOpedDrain)
      return GPIO_MODE_INPUT_OUTPUT_OD;

    return GPIO_MODE_DISABLE;
  }

  [[nodiscard]] auto init() const noexcept -> bool {  // NOLINT
    gpio_config_t const config = {
        .pin_bit_mask = 1ULL << esp_pin,
        .mode = get_esp_mode(),
        .pull_up_en = Inverse ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = Inverse ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (esp_err_t const err = gpio_config(&config); err != ESP_OK) [[unlikely]]
      return false;

    return true;
  }

  [[nodiscard]] auto get_level() const noexcept -> bool {  // NOLINT
    int const level = gpio_get_level(esp_pin);
    return Inverse ? level == 0 : level == 1;
  }

  [[nodiscard]] auto set_level(bool const level) const noexcept -> bool {  // NOLINT
    return gpio_set_level(esp_pin, level) == ESP_OK;
  }

  [[nodiscard]] auto enable() const noexcept -> bool {  // NOLINT
    auto const level = Inverse ? 0 : 1;
    return set_level(level);
  }

  [[nodiscard]] auto disable() const noexcept -> bool {  // NOLINT
    auto const level = Inverse ? 1 : 0;
    return set_level(level);
  }
};

}  // namespace driver
