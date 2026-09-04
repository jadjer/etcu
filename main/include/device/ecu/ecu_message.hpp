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
// Created by jadjer on 3.09.26.
//

#pragma once

#include <array>
#include <numeric>

namespace device {

enum class ECUMode : std::uint8_t {
  INIT = 0x00,
  WAKE_UP = 0xFF,
  READ_TABLE = 0x71,
  READ_RANGE = 0x72,
  UNKNOWN_1 = 0x73,
  UNKNOWN_2 = 0x74,
};

enum class ECUMessageError {
  NONE,
  WRONG_LENGTH,
  WRONG_CHECKSUM,
};

template <std::size_t PayloadSize = 0>
struct ECUMessage {
  static constexpr std::size_t header_size{3};
  static constexpr std::size_t checksum_size{1};
  static constexpr std::size_t total_size{header_size + PayloadSize + checksum_size};
  static constexpr std::uint8_t message_length{common::as_byte(total_size)};

  std::uint8_t address{};
  std::uint8_t length{};
  std::uint8_t mode{};
  std::array<std::uint8_t, PayloadSize> payload{};
  std::uint8_t checksum{};

  ECUMessageError error{ECUMessageError::NONE};

  constexpr ECUMessage() noexcept = default;

  constexpr explicit ECUMessage(std::uint8_t const address, ECUMode const mode, std::array<std::uint8_t, PayloadSize> const& payload = {}) noexcept
      : address{address}, length(message_length), mode{common::as_byte(mode)}, payload{payload}, checksum{calculate_checksum()} {}

  constexpr explicit ECUMessage(std::array<std::uint8_t, total_size> const& bytes) noexcept {
    if (bytes[1] != total_size) [[unlikely]] {
      error = ECUMessageError::WRONG_LENGTH;
      return;
    }

    address = bytes[0];
    length = bytes[1];
    mode = bytes[2];

    if constexpr (PayloadSize > 0)
      std::copy(bytes.begin() + header_size, bytes.begin() + (header_size + PayloadSize), payload.begin());

    checksum = bytes[total_size - 1];

    if (checksum != calculate_checksum()) [[unlikely]] {
      error = ECUMessageError::WRONG_CHECKSUM;
      return;
    }
  }

  [[nodiscard]] constexpr auto is_valid() const noexcept -> bool { return error == ECUMessageError::NONE; }

  [[nodiscard]] constexpr auto calculate_checksum() const noexcept -> std::uint8_t {
    std::uint32_t sum = address + length + mode;
    if constexpr (PayloadSize > 0) {
      sum = std::accumulate(payload.begin(), payload.end(), sum);
    }
    return common::as_byte(0U - sum);
  }

  [[nodiscard]] constexpr auto to_array() const noexcept -> std::array<std::uint8_t, total_size> {
    std::array<std::uint8_t, total_size> bytes{address, length, mode};

    if constexpr (PayloadSize > 0) {
      std::copy(payload.begin(), payload.end(), bytes.begin() + header_size);
    }

    bytes[total_size - 1] = calculate_checksum();

    return bytes;
  }

  [[nodiscard]] constexpr auto operator==(ECUMessage const&) const noexcept -> bool = default;
};

}  // namespace device
