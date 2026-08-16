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

#include <esp_timer.h>
#include "types.hpp"

namespace devices {

template <gpio_num_t Pin>
class Indicator {
 public:
  auto init() noexcept -> SystemError { // NOLINT
    gpio_config_t const config = {
        .pin_bit_mask = 1ULL << Pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (auto const err = gpio_config(&config); err != ESP_OK) {
      return SystemError::IndicatorInitFault;
    }

    return SystemError::None;
  }

  auto update() noexcept -> SystemError { // NOLINT
    return SystemError::None;
  }

  // auto set_status(Mode const  /*mode*/) noexcept -> SystemError {
  //   gpio_set_level(Pin, (esp_timer_get_time() / 100000) % 2);  // Частое мигание (Авария)
  //
  //   return SystemError::None;
  // }
};

}  // namespace devices
