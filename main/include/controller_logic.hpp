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
#include "constants.hpp"
#include "type.hpp"

template <type::Position throttleExpoFactor>
consteval auto generate_expo_lut() -> std::array<type::Position, 101> {
  std::array<type::Position, 101> lut;

  for (std::size_t i = 0; i <= 100; ++i) {
    type::Position constexpr positionMinimal{0};
    type::Position constexpr positionMaximal{100};

    type::Position const in{i};
    type::Position const quadratic_part{(in * in + 50) / 100};

    lut[i] = common::map_range(throttleExpoFactor, positionMinimal, positionMaximal, in, quadratic_part);
  }

  return lut;
}

class Logic {
  static std::array<type::Position, 101> constexpr MExpoLut{generate_expo_lut<constants::system::THROTTLE_EXPO_FACTOR>()};

 public:
  constexpr Logic() noexcept = default;

  [[nodiscard]] auto calculate_servo_position(type::Position const acc_offset,  // NOLINT
                                              type::Position const acc_position,
                                              type::Speed const current_speed,
                                              type::Speed const target_speed) const noexcept -> type::Position {
    return MExpoLut[acc_position.get()];
  }
};
