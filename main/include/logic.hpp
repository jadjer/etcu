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

#include "common/map_range.hpp"
#include "type.hpp"

template <type::Position throttleExpoFactor>
consteval auto generate_expo_lut() -> std::array<type::Position, 101> {
  std::array<type::Position, 101> lut{};

  for (std::size_t i = 0; i <= 100; ++i) {
    type::Position const in{i};

    int64_t const cubic_raw = in * in * in.get() / 10000;
    type::Position const cubic_part{cubic_raw};

    lut[i] = commons::map_range(throttleExpoFactor, type::Position{0}, type::Position{100}, in, cubic_part);
  }

  return lut;
}

class Logic {
  static type::Position constexpr ThrottleExpoFactor = 50;
  static constexpr std::array<type::Position, 101> MExpoLut = generate_expo_lut<ThrottleExpoFactor>();

  constexpr auto apply_throttle_expo(type::Position const input_percent) noexcept -> type::Position {  // NOLINT
    if (input_percent <= 0)
      return 0;

    if (input_percent >= 100)
      return 100;

    type::Position const cubic_part = input_percent * input_percent * input_percent / 10000;

    return commons::map_range<type::Position, type::Position>(ThrottleExpoFactor, 0, 100, input_percent, cubic_part);
  }

 public:
  [[nodiscard]] auto calculate_servo_position(type::Position const acc_offset,
                                              type::Position const acc_position,
                                              type::Speed const current_speed,
                                              type::Speed const target_speed) noexcept -> type::Position {
    return apply_throttle_expo(acc_position);
  }
};
