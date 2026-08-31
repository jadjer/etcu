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
