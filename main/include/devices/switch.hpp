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
