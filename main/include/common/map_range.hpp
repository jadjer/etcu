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

namespace common {

template <typename T>
concept IsBoundedConcept = requires(T instance) {
  requires std::constructible_from<T, std::int32_t>;
  { T::value_min } -> std::convertible_to<std::int32_t>;
  { T::value_max } -> std::convertible_to<std::int32_t>;
  { instance.value } -> std::convertible_to<std::int32_t>;
  { instance = std::int32_t{} } -> std::same_as<T&>;
};

template <typename In, typename Out>
  requires IsBoundedConcept<In> && IsBoundedConcept<Out>
constexpr auto map_range(In const value, In const fromMin, In const fromMax, Out const toMin, Out const toMax) -> Out {
  if (fromMax < fromMin) [[unlikely]] {
    return toMin;
  }

  if (toMax < toMin) [[unlikely]] {
    return toMin;
  }

  In const clamped = std::clamp(value, fromMin, fromMax);

  std::int64_t const v_raw = clamped.value;
  std::int64_t const f_min = fromMin.value;
  std::int64_t const f_max = fromMax.value;
  std::int64_t const t_min = toMin.value;
  std::int64_t const t_max = toMax.value;

  std::int64_t const from_span = f_max - f_min;
  std::int64_t const to_span = t_max - t_min;

  std::int64_t const scaled_raw = t_min + ((v_raw - f_min) * to_span + (from_span / 2)) / from_span;

  return static_cast<Out>(scaled_raw);
}

}  // namespace common
