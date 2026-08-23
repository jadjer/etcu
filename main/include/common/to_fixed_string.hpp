//
// Created by jadjer on 23.08.26.
//

#pragma once

#include <array>

namespace common {

template <std::size_t N>
constexpr auto to_fixed_string(char const* str) noexcept -> std::array<char, N> {
  std::array<char, N> arr{};
  std::size_t i = 0;
  // Копируем символы, пока не дойдем до конца строки или не кончится массив
  while (str[i] != '\0' && i < (N - 1)) {
    arr[i] = str[i];
    i++;
  }
  arr[i] = '\0'; // Гарантируем нуль-терминатор в конце
  return arr;
}

}
