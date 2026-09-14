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
  requires std::constructible_from<T, float> || std::constructible_from<T, std::int32_t>;
  requires std::three_way_comparable<T>;
  { instance.value } -> std::convertible_to<std::int32_t>;
  { instance.template as<float>() } -> std::same_as<float>;
};

template <typename In>
  requires IsBoundedConcept<In>
constexpr auto range(In const value, In const fromMin, In const fromMax) -> In {
  if (fromMax <= fromMin) [[unlikely]] {
    return fromMin;
  }

  return std::clamp(value, fromMin, fromMax, [](In const& a, In const& b) { return a.template as<float>() < b.template as<float>(); });
}

template <typename In, typename Out>
  requires IsBoundedConcept<In> && IsBoundedConcept<Out>
constexpr auto map_range(In const value, In const fromMin, In const fromMax, Out const toMin, Out const toMax) -> Out {
  if (fromMax <= fromMin) [[unlikely]] {
    return toMin;
  }

  if (toMax <= toMin) [[unlikely]] {
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

  std::int64_t const scaled_raw = t_min + ((v_raw - f_min) * to_span + from_span / 2) / from_span;

  return static_cast<Out>(scaled_raw);
}

namespace range_tests {

// ИСПРАВЛЕНИЕ: Один универсальный конструктор на концептах убирает двусмысленность для clangd
template <typename T, T MinVal, T MaxVal>
struct DummyBounded {
  static constexpr T value_min{MinVal};
  static constexpr T value_max{MaxVal};

  T value;

  constexpr DummyBounded(std::convertible_to<float> auto const val) noexcept
      : value{static_cast<T>(std::clamp(static_cast<float>(val), static_cast<float>(value_min), static_cast<float>(value_max)))} {}

  template <typename R>
  [[nodiscard]] constexpr auto as() const noexcept -> R {
    return static_cast<R>(value);
  }

  constexpr auto operator<=>(DummyBounded const&) const = default;
};

using TestBounded = DummyBounded<std::uint16_t, 600, 1250>;

// 1. Проверяем валидацию концепта компилятором
static_assert(IsBoundedConcept<TestBounded>, "CRITICAL: IsBoundedConcept failed to validate TestBounded structure!");

// 2. Проверяем рантайм-логику функции range на этапе compile-time
constexpr TestBounded val1{1000};
constexpr TestBounded val2{-1000};
constexpr TestBounded val3{0};
constexpr TestBounded min_gate{600};
constexpr TestBounded max_gate{900};

static_assert(range(val1, min_gate, max_gate) == max_gate, "Range compile-time test with .as<float>() failed!");
static_assert(range(val2, min_gate, max_gate) == min_gate, "Range compile-time test with .as<float>() failed!");
static_assert(range(val3, min_gate, max_gate) == min_gate, "Range compile-time test with .as<float>() failed!");

// 3. Тест для верификации функции масштабирования map_range
using OutBounded = DummyBounded<std::int32_t, 0, 100>;
constexpr TestBounded input_pos{925};
constexpr TestBounded in_min{600};
constexpr TestBounded in_max{1250};
constexpr OutBounded out_min{0};
constexpr OutBounded out_max{100};

static_assert(map_range(input_pos, in_min, in_max, out_min, out_max) == 50, "Map_range compile-time test failed!");

}  // namespace range_tests

}  // namespace common
