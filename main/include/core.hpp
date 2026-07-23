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

#include <algorithm>
#include <cmath>
#include "configs/configs.hpp"
#include "types.hpp"

class Core {
 public:
  auto init() noexcept -> void {}

  [[nodiscard]] auto calculate_servo_position(Mode mode, float acc_percent, bool cruise_active, float current_speed, float target_speed) noexcept -> float {
    if (mode == Mode::Off) {
      return 0.0f;
    }

    float base_target = acc_percent;

    if (cruise_active) {
      float const speed_error = target_speed - current_speed;
      float const cruise_modifier = speed_error * 2.5f;
      base_target = std::clamp(base_target + cruise_modifier, 0.0f, 100.0f);
    }

    return std::clamp(base_target, 0.0f, 100.0f);
  }
};
