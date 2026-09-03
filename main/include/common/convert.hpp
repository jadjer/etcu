//
// Created by jadjer on 1.09.26.
//

#pragma once

namespace common {

inline auto calculateValueDivide256(std::uint8_t const value) -> float {
  // convert to dec, multiple by 5, then divide result by 256
  // used for TPS Volt, ECT Volt, IAT Volt, MAP Volt
  return static_cast<float>(value) * 5 / 256;
}

inline auto calculateValueMinus40(std::uint8_t const value) -> std::uint8_t {
  // value minus 40
  // used for ECT Temp, IAT Temp
  return value - 40;
}

inline auto calculateValueDivide16(std::uint8_t const value) -> float {
  // value divided by 16 and times 10
  // used for TPS%
  return static_cast<float>(value) * 10 / 16;
}

inline auto calculateValueDivide10(std::uint8_t const value) -> std::uint8_t {
  return value / 10;
}

inline auto calculateValueMultiply10(std::uint8_t const value) -> std::uint16_t {
  return static_cast<std::uint16_t>(value) * 10;
}

}  // namespace common
