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

template <typename T, T minVal>
struct BoundedValueStorage {
  T m_value{minVal};
};

template <typename T, T minVal, T maxVal, typename Tag>
class BoundedValue : public BoundedValueStorage<T, minVal> {
  using BoundedValueStorage<T, minVal>::m_value;

 public:
  constexpr explicit BoundedValue() noexcept = default;

  constexpr explicit BoundedValue(int64_t const val) noexcept {
    auto const clamped = std::clamp(val, static_cast<int64_t>(minVal), static_cast<int64_t>(maxVal));
    BoundedValueStorage<T, minVal>::m_value = static_cast<T>(clamped);
  }

  template <typename OtherT, OtherT otherMin, OtherT otherMax, typename OtherTag>
    requires(!std::same_as<Tag, OtherTag>)
  constexpr explicit BoundedValue(BoundedValue<OtherT, otherMin, otherMax, OtherTag> const&) = delete;

  [[nodiscard]] constexpr T get() const noexcept { return BoundedValueStorage<T, minVal>::m_value; }

  [[nodiscard]] constexpr bool operator==(BoundedValue const& other) const noexcept { return get() == other.get(); }
  [[nodiscard]] constexpr auto operator<=>(BoundedValue const& other) const noexcept { return get() <=> other.get(); }

  [[nodiscard]] constexpr BoundedValue operator+(BoundedValue const& other) const noexcept {
    return BoundedValue{static_cast<int64_t>(get()) + static_cast<int64_t>(other.get())};
  }
  [[nodiscard]] constexpr BoundedValue operator-(BoundedValue const& other) const noexcept {
    return BoundedValue{static_cast<int64_t>(get()) - static_cast<int64_t>(other.get())};
  }
  [[nodiscard]] constexpr BoundedValue operator*(BoundedValue const& other) const noexcept {
    return BoundedValue{static_cast<int64_t>(get()) * static_cast<int64_t>(other.get())};
  }
  [[nodiscard]] constexpr BoundedValue operator/(BoundedValue const& other) const noexcept {
    return BoundedValue{static_cast<int64_t>(get()) / static_cast<int64_t>(other.get())};
  }

  [[nodiscard]] constexpr BoundedValue operator+(std::integral auto const other) const noexcept { return BoundedValue{static_cast<int64_t>(get()) + other}; }
  [[nodiscard]] constexpr BoundedValue operator-(std::integral auto const other) const noexcept { return BoundedValue{static_cast<int64_t>(get()) - other}; }
  [[nodiscard]] constexpr BoundedValue operator*(std::integral auto const other) const noexcept { return BoundedValue{static_cast<int64_t>(get()) * other}; }
  [[nodiscard]] constexpr BoundedValue operator/(std::integral auto const other) const noexcept { return BoundedValue{static_cast<int64_t>(get()) / other}; }
};

template <typename T, T minVal, T maxVal, typename Tag>
[[nodiscard]] constexpr bool operator<=(BoundedValue<T, minVal, maxVal, Tag> const& lhs, std::integral auto const rhs) noexcept {
  return static_cast<int64_t>(lhs.get()) <= static_cast<int64_t>(rhs);
}

template <typename T, T minVal, T maxVal, typename Tag>
[[nodiscard]] constexpr bool operator>=(BoundedValue<T, minVal, maxVal, Tag> const& lhs, std::integral auto const rhs) noexcept {
  return static_cast<int64_t>(lhs.get()) >= static_cast<int64_t>(rhs);
}

template <typename T, T minVal, T maxVal, typename Tag>
[[nodiscard]] constexpr bool operator>(BoundedValue<T, minVal, maxVal, Tag> const& lhs, std::integral auto const rhs) noexcept {
  return static_cast<int64_t>(lhs.get()) > static_cast<int64_t>(rhs);
}

template <typename T, T minVal, T maxVal, typename Tag>
[[nodiscard]] constexpr bool operator<(BoundedValue<T, minVal, maxVal, Tag> const& lhs, std::integral auto const rhs) noexcept {
  return static_cast<int64_t>(lhs.get()) < static_cast<int64_t>(rhs);
}

template <typename T, T minVal, T maxVal, typename Tag>
[[nodiscard]] constexpr bool operator<=(std::integral auto const lhs, BoundedValue<T, minVal, maxVal, Tag> const& rhs) noexcept {
  return static_cast<int64_t>(lhs) <= static_cast<int64_t>(rhs.get());
}

template <typename T, T minVal, T maxVal, typename Tag>
[[nodiscard]] constexpr bool operator>=(std::integral auto const lhs, BoundedValue<T, minVal, maxVal, Tag> const& rhs) noexcept {
  return static_cast<int64_t>(lhs) >= static_cast<int64_t>(rhs.get());
}

template <typename T, T minVal, T maxVal, typename Tag>
[[nodiscard]] constexpr bool operator>(std::integral auto const lhs, BoundedValue<T, minVal, maxVal, Tag> const& rhs) noexcept {
  return static_cast<int64_t>(lhs) > static_cast<int64_t>(rhs.get());
}

template <typename T, T minVal, T maxVal, typename Tag>
[[nodiscard]] constexpr bool operator<(std::integral auto const lhs, BoundedValue<T, minVal, maxVal, Tag> const& rhs) noexcept {
  return static_cast<int64_t>(lhs) < static_cast<int64_t>(rhs.get());
}

}  // namespace commons
