//
// Created by jadjer on 18.08.26.
//

#pragma once

namespace driver {

using GPIOMode = gpio_mode_t;
using GPIOConfig = gpio_config_t;

template <type::GPIONum pin, GPIOMode mode, bool inverse = false>
class GPIO {
 public:
  [[nodiscard]] auto init() const noexcept -> bool {  // NOLINT
    GPIOConfig const config = {
        .pin_bit_mask = 1ULL << pin,
        .mode = mode,
        .pull_up_en = inverse ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = inverse ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (auto const err = gpio_config(&config); err != ESP_OK)
      return false;

    return true;
  }

  [[nodiscard]] auto get_level() const noexcept -> bool {  // NOLINT
    int const level = gpio_get_level(pin);
    return inverse ? level == 0 : level == 1;
  }

  [[nodiscard]] auto set_level(bool const level) const noexcept -> bool {  // NOLINT
    return gpio_set_level(pin, level) == ESP_OK;
  }

  [[nodiscard]] auto enable() const noexcept -> bool {
    auto const level = inverse ? 0 : 1;
    return set_level(level);
  }

  [[nodiscard]] auto disable() const noexcept -> bool {
    auto const level = inverse ? 1 : 0;
    return set_level(level);
  }
};

}  // namespace driver
