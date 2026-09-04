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
// Created by jadjer on 1.09.26.
//

#pragma once

namespace common {

inline auto calculateValueDivide256(std::uint8_t const value) -> std::uint8_t {
  return (static_cast<std::uint16_t>(value) * 5) / 256;
}

inline auto calculateValueMinus40(std::uint8_t const value) -> std::uint8_t {
  return value - 40;
}

inline auto calculateValueDivide16(std::uint8_t const value) -> std::uint8_t {
  return static_cast<std::uint16_t>(value) * 10 / 16;
}

inline auto calculateValueDivide10(std::uint8_t const value) -> std::uint8_t {
  return value / 10;
}

inline auto calculateValueMultiply10(std::uint8_t const value) -> std::uint16_t {
  return static_cast<std::uint16_t>(value) * 10;
}

}  // namespace common
