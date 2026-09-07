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
// Created by jadjer on 4.09.26.
//

#pragma once

#include <array>
#include <numeric>

namespace device {

enum class ServoInstruction : std::uint8_t {
  InstPing = 0x01,
  InstRead = 0x02,
  InstWrite = 0x03,
  InstRegWrite = 0x04,
  InstAction = 0x05,
  InstReset = 0x06,
};

enum class ServoMessageError {
  NONE,
  WRONG_HEADER,
  WRONG_ID,
  WRONG_LENGTH,
  WRONG_CHECKSUM,
  STATUS_ERROR,
};

template <std::size_t PayloadSize = 0>
struct ServoMessage {
  static constexpr std::size_t header_size{5};
  static constexpr std::size_t payload_size{PayloadSize};
  static constexpr std::size_t checksum_size{1};
  static constexpr std::size_t total_size{header_size + PayloadSize + checksum_size};
  static constexpr std::size_t length{payload_size + 2};

  std::uint8_t servo_id{};
  std::uint8_t instruction_or_status{};
  std::array<std::uint8_t, PayloadSize> payload{};

  ServoMessageError error{ServoMessageError::NONE};

  constexpr ServoMessage() noexcept = default;

  constexpr explicit ServoMessage(std::uint8_t const servo_id,
                                  ServoInstruction const instruction,
                                  std::array<std::uint8_t, PayloadSize> const& payload = {}) noexcept
      : servo_id{servo_id}, instruction_or_status{common::as_byte(instruction)}, payload{payload} {}

  constexpr explicit ServoMessage(std::array<std::uint8_t, total_size> const& bytes) noexcept {
    static constexpr std::uint8_t start_byte{0xFF};
    static constexpr std::uint8_t broadcast_id{0xFE};

    if (bytes[0] != start_byte || bytes[1] != start_byte) [[unlikely]] {
      error = ServoMessageError::WRONG_HEADER;
      return;
    }

    if (bytes[2] == broadcast_id) [[unlikely]] {
      error = ServoMessageError::WRONG_ID;
      return;
    }

    if (bytes[3] != length) [[unlikely]] {
      error = ServoMessageError::WRONG_LENGTH;
      return;
    }

    if (bytes[4] != 0) [[unlikely]] {
      error = ServoMessageError::STATUS_ERROR;
      return;
    }

    servo_id = bytes[2];
    instruction_or_status = bytes[4];

    if constexpr (payload_size > 0) {
      std::copy(bytes.begin() + header_size, bytes.begin() + (header_size + payload_size), payload.begin());
    }

    if (std::uint8_t const checksum = bytes[total_size - 1]; checksum != calculate_checksum()) [[unlikely]] {
      error = ServoMessageError::WRONG_CHECKSUM;
      return;
    }
  }

  [[nodiscard]] constexpr auto is_valid() const noexcept -> bool { return error == ServoMessageError::NONE; }

  [[nodiscard]] constexpr auto calculate_checksum() const noexcept -> std::uint8_t {
    std::uint32_t sum = servo_id + length + instruction_or_status;

    if constexpr (payload_size > 0) {
      sum = std::accumulate(payload.begin(), payload.end(), sum);
    }

    return common::as_byte(~sum);
  }

  [[nodiscard]] constexpr auto to_array() const noexcept -> std::array<std::uint8_t, total_size> {
    std::array<std::uint8_t, total_size> bytes{0xFF, 0xFF, servo_id, length, instruction_or_status};

    if constexpr (payload_size > 0) {
      std::copy(payload.begin(), payload.end(), bytes.begin() + header_size);
    }

    bytes[total_size - 1] = calculate_checksum();

    return bytes;
  }

  [[nodiscard]] constexpr auto operator==(ServoMessage const&) const noexcept -> bool = default;
};

}  // namespace device
