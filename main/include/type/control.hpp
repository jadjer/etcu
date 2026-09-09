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

namespace type {

struct PositionRange {
  Position min{Position::value_min};
  Position max{Position::value_max};

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
  static constexpr std::uint32_t current_version{1};

  std::uint32_t version{0};

  PositionRange servo{};
  PositionRange accelerator{};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::ControlDTO {
    return dto::ControlDTO{
        .servo = servo.to_dto(),
        .accelerator = accelerator.to_dto(),
    };
  }

  [[nodiscard]] auto operator<=>(Control const&) const = default;
};

}  // namespace type
