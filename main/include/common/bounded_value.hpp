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
// Created by jadjer on 28.08.26.
//

#pragma once

#include <algorithm>
#include <concepts>

namespace common {

template <typename T, T MinVal, T MaxVal>
  requires std::convertible_to<T, float> && (MinVal <= MaxVal)
struct BoundedValue {
  static constexpr T value_min{MinVal};
  static constexpr T value_max{MaxVal};

  T value{value_min};

  constexpr BoundedValue(std::integral auto const val) noexcept // NOLINT
    : value{static_cast<T>(std::clamp(static_cast<T>(val), value_min, value_max))} {}

  constexpr BoundedValue(std::floating_point auto const val) noexcept  // NOLINT
      : value{static_cast<T>(std::clamp(static_cast<T>(val), value_min, value_max))} {}

  constexpr BoundedValue() = default;

  constexpr auto operator=(BoundedValue const& other) noexcept -> BoundedValue& = default;

  constexpr auto get() const noexcept -> T { return value; }

  template <typename R>
    requires std::convertible_to<T, R>
  constexpr auto as() const noexcept -> R {
    return static_cast<R>(value);
  }

  constexpr auto operator<=>(BoundedValue const&) const = default;

  constexpr auto operator*(BoundedValue const& other) const noexcept -> BoundedValue { return BoundedValue{static_cast<float>(value) * static_cast<float>(other.value)}; }
  constexpr auto operator/(BoundedValue const& other) const noexcept -> BoundedValue { return BoundedValue{static_cast<float>(value) / static_cast<float>(other.value)}; }
  constexpr auto operator+(BoundedValue const& other) const noexcept -> BoundedValue { return BoundedValue{static_cast<float>(value) + static_cast<float>(other.value)}; }
  constexpr auto operator-(BoundedValue const& other) const noexcept -> BoundedValue { return BoundedValue{static_cast<float>(value) - static_cast<float>(other.value)}; }
};

}  // namespace common
