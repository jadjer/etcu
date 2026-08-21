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
// Created by jadjer on 17.08.26.
//

#pragma once

#include <algorithm>

namespace common {

template <typename T, T minVal, T maxVal, typename Tag>
  requires std::is_arithmetic_v<T> && std::convertible_to<T, std::int32_t> && (minVal <= maxVal)

struct BoundedValue {
  using value_type = T;
  static constexpr T min_value = minVal;
  static constexpr T max_value = maxVal;

  std::int32_t value{static_cast<std::int32_t>(minVal)};

  constexpr BoundedValue() noexcept = default;

  constexpr BoundedValue(std::int32_t const val) noexcept {  // NOLINT
    value = std::clamp(val, static_cast<std::int32_t>(minVal), static_cast<std::int32_t>(maxVal));
  }

  template <typename OtherT, OtherT oMin, OtherT oMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr BoundedValue(BoundedValue<OtherT, oMin, oMax, OtherTag> const&) = delete;  // NOLINT

  constexpr auto operator=(std::int32_t const val) noexcept -> BoundedValue& {
    value = std::clamp(val, static_cast<std::int32_t>(minVal), static_cast<std::int32_t>(maxVal));

    return *this;
  }

  constexpr auto operator<=>(BoundedValue const&) const = default;
};

}  // namespace common
