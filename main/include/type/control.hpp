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

struct RPMRange {
  RPM min{};
  RPM max{};

  [[nodiscard]] constexpr auto contains(RPM const& current_rpm) const noexcept -> bool { return current_rpm >= min && current_rpm <= max; }

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::RPMRangeDTO {
    return dto::RPMRangeDTO{
        .min = min.get(),
        .max = max.get(),
    };
  }

  [[nodiscard]] auto operator<=>(RPMRange const&) const = default;
};

struct SpeedRange {
  Speed min{};
  Speed max{};

  [[nodiscard]] constexpr auto contains(Speed const& current_speed) const noexcept -> bool { return current_speed >= min && current_speed <= max; }

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::SpeedRangeDTO {
    return dto::SpeedRangeDTO{
        .min = min.get(),
        .max = max.get(),
    };
  }

  [[nodiscard]] auto operator<=>(SpeedRange const&) const = default;
};

struct Cruise {
  float p{};
  float i{};
  float d{};
  RPMRange rpm{};
  Position limiter{};
  SpeedRange speed{};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::CruiseDTO {
    return dto::CruiseDTO{
        .p = p,
        .i = i,
        .d = d,
        .rpm = rpm.to_dto(),
        .speed = speed.to_dto(),
        .limiter = limiter.get(),
    };
  }

  [[nodiscard]] auto operator<=>(Cruise const&) const = default;
};

struct PositionRange {
  Position min{};
  Position max{};

  [[nodiscard]] constexpr auto contains(Position const& current_position) const noexcept -> bool { return current_position >= min && current_position <= max; }

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::PositionRangeDTO {
    return dto::PositionRangeDTO{
        .min = min.get(),
        .max = max.get(),
    };
  }

  [[nodiscard]] auto operator<=>(PositionRange const&) const = default;
};

struct Control {
  static constexpr std::string_view name{"control"};
  static constexpr std::uint32_t current_version{2};

  std::uint32_t version{};

  Cruise cruise{};
  PositionRange servo{};
  PositionRange accelerator{};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::ControlDTO {
    return dto::ControlDTO{
        .cruise = cruise.to_dto(),
        .servo = servo.to_dto(),
        .accelerator = accelerator.to_dto(),
    };
  }

  [[nodiscard]] auto operator<=>(Control const&) const = default;
};

}  // namespace type
