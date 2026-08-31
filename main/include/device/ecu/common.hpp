//
// Created by jadjer on 24.08.22.
//

#pragma once

namespace common {

/**
 * @brief
 * @param value
 * @return
 */
inline float calcValueDivide256(std::uint8_t const value) {
  // convert to dec, multiple by 5, then divide result by 256
  // used for TPS Volt, ECT Volt, IAT Volt, MAP Volt
  return static_cast<float>(value) * 5 / 256;
}

/**
 * @brief
 * @param value
 * @return
 */
inline uint8_t calcValueMinus40(std::uint8_t const value) {
  // value minus 40
  // used for ECT Temp, IAT Temp
  return value - 40;
}

/**
 * @brief
 * @param value
 * @return
 */
inline float calcValueDivide16(std::uint8_t const value) {
  // value divided by 16 and times 10
  // used for TPS%
  return static_cast<float>(value) * 10 / 16;
}

/**
 * @brief
 * @param value
 * @return
 */
inline float calcValueDivide10(std::uint8_t const value) {
  return static_cast<float>(value) / 10;
}

/**
 * @brief
 * @param data
 * @param len
 * @return
 */
inline std::uint16_t calcChecksum(std::uint8_t const* data, std::size_t const len) {
  uint16_t checkSum = 0;

  for (size_t i = 0; i < len; i++) {
    checkSum -= data[i];
  }

  return checkSum;
}

}  // namespace common
