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

  constexpr BoundedValue(std::convertible_to<float> auto const val) noexcept  // NOLINT
      : value{static_cast<T>(std::clamp(static_cast<float>(val), static_cast<float>(value_min), static_cast<float>(value_max)))} {}

  constexpr BoundedValue() = default;

  constexpr auto operator=(BoundedValue const& other) noexcept -> BoundedValue& = default;

  [[nodiscard]] constexpr auto get() const noexcept -> T { return value; }

  template <typename R>
    requires std::convertible_to<T, R>
  [[nodiscard]] constexpr auto as() const noexcept -> R {
    return static_cast<R>(value);
  }

  constexpr auto operator<=>(BoundedValue const&) const noexcept = default;

  constexpr auto operator*(BoundedValue const& other) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<float>(value) * static_cast<float>(other.value)};
  }
  constexpr auto operator/(BoundedValue const& other) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<float>(value) / static_cast<float>(other.value)};
  }
  constexpr auto operator+(BoundedValue const& other) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<float>(value) + static_cast<float>(other.value)};
  }
  constexpr auto operator-(BoundedValue const& other) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<float>(value) - static_cast<float>(other.value)};
  }

  constexpr auto operator<=>(std::convertible_to<float> auto const other) const noexcept { return static_cast<float>(value) <=> static_cast<float>(other); }
  constexpr auto operator*(std::convertible_to<float> auto const other) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<float>(value) * static_cast<float>(other)};
  }
  constexpr auto operator/(std::convertible_to<float> auto const other) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<float>(value) / static_cast<float>(other)};
  }
  constexpr auto operator+(std::convertible_to<float> auto const other) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<float>(value) + static_cast<float>(other)};
  }
  constexpr auto operator-(std::convertible_to<float> auto const other) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<float>(value) - static_cast<float>(other)};
  }
};

namespace bounded_value_tests {

using TestType = BoundedValue<std::uint16_t, 600, 1250>;

static_assert(std::is_trivially_copyable_v<TestType>);
static_assert(sizeof(TestType) == sizeof(std::uint16_t));
static_assert(TestType::value_min == 600);
static_assert(TestType::value_max == 1250);

constexpr TestType empty{};
static_assert(empty.get() == TestType::value_min);
static_assert(empty.get() != TestType::value_max);

constexpr TestType normal_int{800};
static_assert(normal_int.get() == 800);

constexpr TestType normal_float{750.5f};
static_assert(normal_float.get() == 750);

constexpr TestType negative_float{-750.5f};
static_assert(negative_float.get() == TestType::value_min);

constexpr TestType underflow_test{100};
static_assert(underflow_test.get() == TestType::value_min);

constexpr TestType overflow_test{5000};
static_assert(overflow_test.get() == TestType::value_max);

constexpr TestType base{1000};
constexpr float offset_val{100.0f};
static_assert((base + offset_val).get() == 1100);
static_assert((base - offset_val).get() == 900);

constexpr float giant_offset{400.0f};
static_assert((base + giant_offset).get() == TestType::value_max);

constexpr TestType min_base{600};
static_assert((min_base - offset_val).get() == TestType::value_min);

constexpr TestType another_object{600};
static_assert((base + another_object).get() == TestType::value_max);
static_assert((base - another_object).get() == TestType::value_min);

static_assert(base.as<float>() == 1000.0f);
static_assert(min_base < base);
static_assert(base > offset_val);
static_assert(base != offset_val);
static_assert(base == 1000);
}  // namespace bounded_value_tests

}  // namespace common
