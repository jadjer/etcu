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
#include <cstdint>

namespace common {

template <typename T, T minVal, T maxVal, typename Tag>
  requires std::is_arithmetic_v<T> && (minVal <= maxVal)
struct BoundedValue {
  using ConvertableType = std::int64_t;

  static constexpr T MinValue{minVal};
  static constexpr T MaxValue{maxVal};

  T value{minVal};

  constexpr explicit BoundedValue() = default;

  template <typename OtherT, OtherT oMin, OtherT oMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr explicit BoundedValue(BoundedValue<OtherT, oMin, oMax, OtherTag> const&) = delete;
  constexpr explicit BoundedValue(T const& val) noexcept
      : value{static_cast<T>(std::clamp(static_cast<ConvertableType>(val), static_cast<ConvertableType>(minVal), static_cast<ConvertableType>(maxVal)))} {}

  template <typename OtherT, OtherT oMin, OtherT oMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr auto operator=(BoundedValue<OtherT, oMin, oMax, OtherTag> const&) -> BoundedValue& = delete;
  constexpr auto operator=(BoundedValue const& other) noexcept -> BoundedValue& = default;
  constexpr auto operator=(T const& val) noexcept -> BoundedValue& {
    value = static_cast<T>(std::clamp(static_cast<ConvertableType>(val), static_cast<ConvertableType>(minVal), static_cast<ConvertableType>(maxVal)));
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
    return BoundedValue{static_cast<T>(static_cast<ConvertableType>(value) * static_cast<ConvertableType>(other.value))};
  }
  constexpr auto operator*(T const& val) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<T>(static_cast<ConvertableType>(value) * static_cast<ConvertableType>(val))};
  }

  template <typename OtherT, OtherT oMin, OtherT oMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr auto operator/(BoundedValue<OtherT, oMin, oMax, OtherTag> const&) const = delete;
  constexpr auto operator/(BoundedValue const& other) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<T>(static_cast<ConvertableType>(value) / static_cast<ConvertableType>(other.value))};
  }
  constexpr auto operator/(T const& val) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<T>(static_cast<ConvertableType>(value) / static_cast<ConvertableType>(val))};
  }

  template <typename OtherT, OtherT oMin, OtherT oMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr auto operator+(BoundedValue<OtherT, oMin, oMax, OtherTag> const&) const = delete;
  constexpr auto operator+(BoundedValue const& other) const noexcept -> BoundedValue { return BoundedValue{static_cast<T>(value + other.value)}; }
  constexpr auto operator+(T const& val) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<T>(static_cast<ConvertableType>(value) + static_cast<ConvertableType>(val))};
  }

  template <typename OtherT, OtherT oMin, OtherT oMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr auto operator-(BoundedValue<OtherT, oMin, oMax, OtherTag> const&) const = delete;
  constexpr auto operator-(BoundedValue const& other) const noexcept -> BoundedValue { return BoundedValue{static_cast<T>(value - other.value)}; }
  constexpr auto operator-(T const& val) const noexcept -> BoundedValue {
    return BoundedValue{static_cast<T>(static_cast<ConvertableType>(value) - static_cast<ConvertableType>(val))};
  }
};

}  // namespace common
