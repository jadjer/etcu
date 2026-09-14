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

struct AccPositionRange {
  AccPosition min{0};
  AccPosition max{0};

  [[nodiscard]] auto operator<=>(AccPositionRange const&) const = default;
};

struct AcceleratorCalibrationData {
  static constexpr std::string_view name{"acc_calib"};
  static constexpr std::uint32_t current_version{2};

  std::uint32_t version{0};

  AccPositionRange hall_a{};
  AccPositionRange hall_b{};

  [[nodiscard]] auto operator<=>(AcceleratorCalibrationData const&) const = default;
};

struct ServoPositionRange {
  ServoPosition min{0};
  ServoPosition max{0};

  [[nodiscard]] auto operator<=>(ServoPositionRange const&) const = default;
};

struct ServoCalibrationData {
  static constexpr std::string_view name{"servo_calib"};
  static constexpr std::uint32_t current_version{2};

  std::uint32_t version{0};

  ServoPositionRange position{};

  [[nodiscard]] auto operator<=>(ServoCalibrationData const&) const = default;
};

struct Calibration {
  ServoCalibrationData servo{};
  AcceleratorCalibrationData accelerator{};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::CalibrationDTO {
    return dto::CalibrationDTO{
        .hall_a_min = accelerator.hall_a.min.get(),
        .hall_a_max = accelerator.hall_a.max.get(),
        .hall_b_min = accelerator.hall_b.min.get(),
        .hall_b_max = accelerator.hall_b.max.get(),

        .servo_min = servo.position.min.get(),
        .servo_max = servo.position.max.get(),
    };
  }

  [[nodiscard]] auto operator<=>(Calibration const&) const = default;
};

}  // namespace type
