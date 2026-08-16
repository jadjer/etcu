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

namespace commons {

template <concepts::IsBounded In, concepts::IsBounded Out>
constexpr auto map_range(In const value, In const fromMin, In const fromMax, Out const toMin, Out const toMax) -> Out {
  auto const raw_value = static_cast<int64_t>(value.get());
  auto const raw_fromMin = static_cast<int64_t>(fromMin.get());
  auto const raw_fromMax = static_cast<int64_t>(fromMax.get());
  auto const raw_toMin = static_cast<int64_t>(toMin.get());
  auto const raw_toMax = static_cast<int64_t>(toMax.get());

  if (raw_fromMax <= raw_fromMin) [[unlikely]]
    return toMin;

  int64_t const clamped_value = std::clamp(raw_value, raw_fromMin, raw_fromMax);
  int64_t const from_span = raw_fromMax - raw_fromMin;
  int64_t const to_span = raw_toMax - raw_toMin;
  int64_t const scaled_value = raw_toMin + (clamped_value - raw_fromMin) * to_span / from_span;

  return Out(scaled_value);
}

}  // namespace commons
