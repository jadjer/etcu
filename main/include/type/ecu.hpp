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
#include <expected>

namespace type {

enum class MessageError {
  WRONG_LENGTH,
  WRONG_CHECKSUM,
};

template <std::size_t PayloadSize>
struct Message {
  static constexpr std::size_t header_size{3};
  static constexpr std::size_t checksum_size{1};
  static constexpr std::size_t total_size{header_size + PayloadSize + checksum_size};
  static constexpr std::uint8_t calculated_length = static_cast<std::uint8_t>(1U + PayloadSize);

  std::uint8_t address{};
  std::uint8_t length{static_cast<std::uint8_t>(total_size)};
  std::uint8_t mode{};
  std::array<std::uint8_t, PayloadSize> payload{};
  std::uint8_t checksum{};

  [[nodiscard]] auto calculate_checksum() const noexcept -> std::uint8_t {
    std::uint32_t sum = address + length + mode;
    if constexpr (PayloadSize > 0) {
      sum = std::accumulate(payload.begin(), payload.end(), sum);
    }
    return static_cast<std::uint8_t>(0U - sum);
  }

  [[nodiscard]] auto to_array() const noexcept -> std::array<std::uint8_t, total_size> {
    std::array<std::uint8_t, total_size> bytes{};

    bytes[0] = address;
    bytes[1] = length;
    bytes[2] = mode;

    if constexpr (PayloadSize > 0) {
      std::copy(payload.begin(), payload.end(), bytes.begin() + header_size);
    }

    bytes[total_size - 1] = calculate_checksum();

    return bytes;
  }

  [[nodiscard]] static auto from_array(std::array<std::uint8_t, total_size> const& bytes) noexcept -> std::expected<Message, MessageError> {
    if (bytes[1] != total_size) [[unlikely]] {
      ESP_LOGE("MSG", "Length mismatch in packet: got %d, expected %zu", bytes[1], total_size);
      return std::unexpected(MessageError::WRONG_LENGTH);
    }

    Message msg{};

    msg.address = bytes[0];
    msg.length = bytes[1];
    msg.mode = bytes[2];

    if constexpr (PayloadSize > 0) {
      std::copy(bytes.begin() + header_size, bytes.begin() + (header_size + PayloadSize), msg.payload.begin());
    }

    msg.checksum = bytes[total_size - 1];

    std::uint8_t const calculated_checksum = msg.calculate_checksum();
    if (msg.checksum != calculated_checksum) [[unlikely]] {
      ESP_LOGE("MSG", "Checksum mismatch: expected 0x%02X, calculated 0x%02X", msg.checksum, calculated_checksum);
      return std::unexpected(MessageError::WRONG_CHECKSUM);
    }

    return msg;
  }

  [[nodiscard]] auto operator==(Message const&) const noexcept -> bool = default;
};

}  // namespace type
