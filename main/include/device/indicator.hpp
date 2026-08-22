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

#include "concepts.hpp"
#include "type.hpp"

namespace device {

template <class Driver>
  requires concepts::GPIO<Driver>

class Indicator {
  Driver& m_driver;

 public:
  constexpr explicit Indicator(Driver& driver) : m_driver(driver) {}

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_driver.init())
      return type::SystemError::IndicatorInitFault;

    return type::SystemError::None;
  }

  [[nodiscard]] auto update() noexcept -> type::SystemError {  // NOLINT
    return type::SystemError::None;
  }

  // auto set_status(Mode const  /*mode*/) noexcept -> SystemError {
  //   gpio_set_level(Pin, (esp_timer_get_time() / 100000) % 2);  // Частое мигание (Авария)
  //
  //   return SystemError::None;
  // }
};

}  // namespace device
