//
// Created by jadjer on 23.07.26.
//

#pragma once

#include <driver/gpio.h>

namespace devices {

template <gpio_num_t Pin, bool ActiveLow = true>
class Switch {
 public:
  void init() noexcept {
    gpio_config_t const config = {
        .pin_bit_mask = (1ULL << Pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = ActiveLow ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = ActiveLow ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&config);
  }

  [[nodiscard]] auto is_active() const noexcept -> bool {
    int const level = gpio_get_level(Pin);
    return ActiveLow ? (level == 0) : (level == 1);
  }
};

}  // namespace devices
