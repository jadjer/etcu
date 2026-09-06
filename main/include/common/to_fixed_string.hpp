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
// Created by jadjer on 23.08.26.
//

#pragma once

#include <array>

namespace common {

template <std::size_t StringSize>
constexpr auto to_fixed_string(char const* str) noexcept -> std::array<char, StringSize> {
  static constexpr std::size_t string_size{StringSize};

  std::array<char, string_size> array{};
  std::size_t i = 0;

  while (str[i] != '\0' && i < (string_size - 1)) {
    array[i] = str[i];
    i++;
  }

  array[i] = '\0';

  return array;
}

}  // namespace common
