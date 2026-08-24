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

template <typename T, T MinVal, T MaxVal>
  requires std::is_convertible_v<T, std::int32_t> && std::is_arithmetic_v<T> && (MinVal <= MaxVal)

struct BoundedValue {
  static constexpr std::int32_t min_value{MinVal};
  static constexpr std::int32_t max_value{MaxVal};

  std::int32_t value{min_value};

  constexpr BoundedValue() = default;
  constexpr BoundedValue(std::int32_t const val) noexcept : value{std::clamp(val, min_value, max_value)} {}

  constexpr auto get() const noexcept -> T { return static_cast<T>(value); }

  constexpr auto operator<=>(BoundedValue const&) const = default;

  constexpr auto operator=(BoundedValue const& other) noexcept -> BoundedValue& = default;

  constexpr auto operator*(BoundedValue const& other) const noexcept -> BoundedValue { return BoundedValue{value * other.value}; }
  constexpr auto operator/(BoundedValue const& other) const noexcept -> BoundedValue { return BoundedValue{value / other.value}; }
  constexpr auto operator+(BoundedValue const& other) const noexcept -> BoundedValue { return BoundedValue{value + other.value}; }
  constexpr auto operator-(BoundedValue const& other) const noexcept -> BoundedValue { return BoundedValue{value - other.value}; }
};

}  // namespace common
