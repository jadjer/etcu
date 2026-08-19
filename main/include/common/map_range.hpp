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
#include "concepts.hpp"

namespace common {

template <class In, class Out>
  requires concepts::IsBounded<In> && concepts::IsBounded<Out>

constexpr auto map_range(In const value, In const fromMin, In const fromMax, Out const toMin, Out const toMax) -> Out {
  if (fromMax <= fromMin) [[unlikely]]
    return toMin;

  if (toMax <= toMin) [[unlikely]]
    return toMin;

  In const clamped_value = std::clamp(value, fromMin, fromMax);

  int64_t const v_raw = clamped_value.get();
  int64_t const f_min = fromMin.get();
  int64_t const f_max = fromMax.get();
  int64_t const t_min = toMin.get();
  int64_t const t_max = toMax.get();

  int64_t const from_span = f_max - f_min;
  int64_t const to_span = t_max - t_min;

  int64_t const scaled_raw = t_min + ((v_raw - f_min) * to_span + from_span / 2) / from_span;

  return Out{scaled_raw};
}

}  // namespace common
