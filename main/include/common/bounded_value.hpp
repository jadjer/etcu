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

template <typename T, T MinVal, T MaxVal, typename Tag>
  requires std::is_arithmetic_v<T> && (MinVal <= MaxVal)
struct BoundedValue {
  using convertible_type = std::int64_t;

  static constexpr T min_value{MinVal};
  static constexpr T max_value{MaxVal};

  T value{min_value};

  constexpr explicit BoundedValue() = default;

  template <typename OtherT, OtherT oMin, OtherT oMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr explicit BoundedValue(BoundedValue<OtherT, oMin, oMax, OtherTag> const&) = delete;
  constexpr explicit BoundedValue(T const& val) noexcept
      : value{static_cast<T>(std::clamp(static_cast<convertible_type>(val), static_cast<convertible_type>(min_value), static_cast<convertible_type>(max_value)))} {}

  template <typename OtherT, OtherT oMin, OtherT oMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr auto operator=(BoundedValue<OtherT, oMin, oMax, OtherTag> const&) -> BoundedValue& = delete;
  constexpr auto operator=(BoundedValue const& other) noexcept -> BoundedValue& = default;
  constexpr auto operator=(T const& val) noexcept -> BoundedValue& {
    value = static_cast<T>(std::clamp(static_cast<convertible_type>(val), static_cast<convertible_type>(min_value), static_cast<convertible_type>(max_value)));
    return *this;
  }

  template <typename OtherT, OtherT oMin, OtherT oMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr auto operator<=>(BoundedValue<OtherT, oMin, oMax, OtherTag> const&) const = delete;
  constexpr auto operator<=>(BoundedValue const&) const = default;

  template <typename OtherT, OtherT oMin, OtherT oMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr auto operator*(BoundedValue<OtherT, oMin, oMax, OtherTag> const&) const = delete;
  constexpr auto operator*(BoundedValue const& other) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<T>(static_cast<convertible_type>(value) * static_cast<convertible_type>(other.value))};
  }
  constexpr auto operator*(T const& val) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<T>(static_cast<convertible_type>(value) * static_cast<convertible_type>(val))};
  }

  template <typename OtherT, OtherT oMin, OtherT oMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr auto operator/(BoundedValue<OtherT, oMin, oMax, OtherTag> const&) const = delete;
  constexpr auto operator/(BoundedValue const& other) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<T>(static_cast<convertible_type>(value) / static_cast<convertible_type>(other.value))};
  }
  constexpr auto operator/(T const& val) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<T>(static_cast<convertible_type>(value) / static_cast<convertible_type>(val))};
  }

  template <typename OtherT, OtherT oMin, OtherT oMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr auto operator+(BoundedValue<OtherT, oMin, oMax, OtherTag> const&) const = delete;
  constexpr auto operator+(BoundedValue const& other) const noexcept -> BoundedValue { return BoundedValue{static_cast<T>(value + other.value)}; }
  constexpr auto operator+(T const& val) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<T>(static_cast<convertible_type>(value) + static_cast<convertible_type>(val))};
  }

  template <typename OtherT, OtherT oMin, OtherT oMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr auto operator-(BoundedValue<OtherT, oMin, oMax, OtherTag> const&) const = delete;
  constexpr auto operator-(BoundedValue const& other) const noexcept -> BoundedValue { return BoundedValue{static_cast<T>(value - other.value)}; }
  constexpr auto operator-(T const& val) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<T>(static_cast<convertible_type>(value) - static_cast<convertible_type>(val))};
  }
};

}  // namespace common
