//
// Created by jadjer on 23.07.26.
//

#pragma once

#include <esp_timer.h>
#include "types.hpp"

namespace devices {

template <gpio_num_t Pin>
class Indicator {
 public:
  auto init() noexcept -> void {
    gpio_config_t const config = {
        .pin_bit_mask = (1ULL << Pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&config);
  }

  auto set_status(Mode const mode, SystemError const err) noexcept -> void {
    if (err != SystemError::None) {
      gpio_set_level(Pin, (esp_timer_get_time() / 100000) % 2);  // Частое мигание (Авария)
    } else if (mode == Mode::Eco) {
      gpio_set_level(Pin, 1);  // Статичное свечение (Круиз)
    } else {
      gpio_set_level(Pin, (esp_timer_get_time() / 1000000) % 2);  // Медленный пульс (Норма)
    }
  }
};

}  // namespace devices
