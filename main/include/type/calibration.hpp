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
// Created by jadjer on 24.08.26.
//

#pragma once

#include <string_view>

#include "type/dto.hpp"
#include "type/type.hpp"

namespace type {

struct AcceleratorCalibrationData {
  static constexpr std::string_view name{"acc_calib"};
  static constexpr std::uint32_t current_version{1};

  std::uint32_t version{0};

  AccPosition hall_a_minimal{0};
  AccPosition hall_a_maximal{0};
  AccPosition hall_b_minimal{0};
  AccPosition hall_b_maximal{0};

  [[nodiscard]] auto operator<=>(AcceleratorCalibrationData const&) const = default;
};

struct ServoCalibrationData {
  static constexpr std::string_view name{"servo_calib"};
  static constexpr std::uint32_t current_version{1};

  std::uint32_t version{0};

  ServoPosition position_minimal{0};
  ServoPosition position_maximal{0};

  [[nodiscard]] auto operator<=>(ServoCalibrationData const&) const = default;
};

struct Calibration {
  ServoCalibrationData servo{};
  AcceleratorCalibrationData accelerator{};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::CalibrationDTO {
    return dto::CalibrationDTO{
        .hall_a_min = accelerator.hall_a_minimal.get(),
        .hall_a_max = accelerator.hall_a_maximal.get(),
        .hall_b_min = accelerator.hall_b_minimal.get(),
        .hall_b_max = accelerator.hall_b_maximal.get(),

        .servo_min = servo.position_minimal.get(),
        .servo_max = servo.position_maximal.get(),
    };
  }

  [[nodiscard]] auto operator<=>(Calibration const&) const = default;
};

}  // namespace type
