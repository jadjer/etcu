//
// Created by jadjer on 2.08.26.
//

#pragma once

#include <algorithm>

namespace commons {

template <typename T, typename S>
  requires std::is_arithmetic_v<T> && std::is_arithmetic_v<S>
constexpr auto map_range(T const value, T const fromMin, T const fromMax, S const toMin, S const toMax) -> S {
  if (fromMax <= fromMin) [[unlikely]] {
    return toMin;
  }

  T const clamped_value = std::clamp(value, fromMin, fromMax);

  auto const from_span = fromMax - fromMin;
  auto const to_span = toMax - toMin;
  auto const scaled_value = toMin + ((clamped_value - fromMin) * to_span / from_span);

  return static_cast<S>(scaled_value);
}

} // namespace commons
