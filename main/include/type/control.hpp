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
// Created by jadjer on 9.09.26.
//

#pragma once

#include "type/dto.hpp"
#include "type/type.hpp"

namespace type {

struct Cruise {
  float p{};
  float i{};
  float d{};
  RPM rpm_min{};
  RPM rpm_max{};
  Speed speed_min{};
  Speed speed_max{};
  float filter_alpha{};
  float fade_duration{};
  float integral_min{};
  float integral_max{};
  Position limiter_left{};
  Position limiter_right{};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::CruiseDTO {
    return dto::CruiseDTO{
        .p = p,
        .i = i,
        .d = d,
        .integral_min = integral_min,
        .integral_max = integral_max,
        .filter_alpha = filter_alpha,
        .fade_duration = fade_duration,
        .rpm_min = rpm_min.get(),
        .rpm_max = rpm_max.get(),
        .speed_min = speed_min.get(),
        .speed_max = speed_max.get(),
        .limiter_left = limiter_left.get(),
        .limiter_right = limiter_right.get(),
    };
  }

  [[nodiscard]] auto operator<=>(Cruise const&) const = default;
};

struct Control {
  static constexpr std::string_view name{"control"};
  static constexpr std::uint32_t current_version{3};

  std::uint32_t version{};

  Cruise cruise{};
  Position servo_min{};
  Position servo_max{};
  Position accelerator_min{};
  Position accelerator_max{};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::ControlDTO {
    return dto::ControlDTO{
        .cruise = cruise.to_dto(),
        .servo_min = servo_min.get(),
        .servo_max = servo_max.get(),
        .accelerator_min = accelerator_min.get(),
        .accelerator_max = accelerator_max.get(),
    };
  }

  [[nodiscard]] auto operator<=>(Control const&) const = default;
};

}  // namespace type
