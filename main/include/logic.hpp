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

#include "commons/map_range.hpp"
#include "types.hpp"

template <Position ThrottleExpoFactor>
consteval auto generate_expo_lut() -> std::array<Position, 101> {
  std::array<Position, 101> lut{};

  for (std::size_t i = 0; i <= 100; ++i) {
    auto const in = static_cast<std::uint64_t>(i);

    std::uint64_t const cubic_part = (in * in * in) / 10000;
    std::uint64_t const linear_part = in;

    lut[i] = commons::map_range<Position, Position>(
      ThrottleExpoFactor,
      0, 100,
      static_cast<Position>(linear_part),
      static_cast<Position>(cubic_part)
    );
  }

  return lut;
}

class Logic {
  static Position constexpr ThrottleExpoFactor = 50;
  static constexpr std::array<Position, 101> MExpoLut = generate_expo_lut<ThrottleExpoFactor>();

  constexpr auto apply_throttle_expo(Position const input_percent) noexcept -> Position {
    if (input_percent <= 0)
      return 0;

    if (input_percent >= 100)
      return 100;

    Position const cubic_part = (input_percent * input_percent * input_percent) / 10000;

    return commons::map_range<Position, Position>(ThrottleExpoFactor, 0, 100, input_percent, cubic_part);
  }

 public:
  [[nodiscard]] auto calculate_servo_position(Position const acc_offset,
                                              Position const acc_position,
                                              Speed const current_speed,
                                              Speed const target_speed) noexcept -> Position {
    return apply_throttle_expo(acc_position);
  }
};
