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
