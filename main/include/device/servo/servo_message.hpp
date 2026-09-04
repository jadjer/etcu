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
  static constexpr std::uint8_t message_length{common::as_byte(payload_size + 2)};

  std::uint8_t servo_id{};
  std::uint8_t length{};
  std::uint8_t instruction_or_status{};
  std::array<std::uint8_t, PayloadSize> payload{};
  std::uint8_t checksum{};

  ServoMessageError error{ServoMessageError::NONE};

  constexpr ServoMessage() noexcept = default;

  constexpr explicit ServoMessage(std::uint8_t const servo_id,
                                  ServoInstruction const instruction,
                                  std::array<std::uint8_t, PayloadSize> const& payload = {}) noexcept
      : servo_id{servo_id}, length{message_length}, instruction_or_status{common::as_byte(instruction)}, payload{payload}, checksum{calculate_checksum()} {}

  constexpr explicit ServoMessage(std::uint8_t const expected_servo_id, std::array<std::uint8_t, total_size> const& bytes) noexcept {
    static constexpr std::uint8_t start_byte{0xFF};
    static constexpr std::uint8_t broadcast_id{0xFE};

    if (bytes[0] != start_byte || bytes[1] != start_byte) [[unlikely]] {
      error = ServoMessageError::WRONG_HEADER;
      return;
    }

    if (bytes[2] != expected_servo_id && expected_servo_id != broadcast_id) [[unlikely]] {
      error = ServoMessageError::WRONG_ID;
      return;
    }

    if (bytes[3] != message_length) [[unlikely]] {
      error = ServoMessageError::WRONG_LENGTH;
      return;
    }

    if (bytes[4] != 0) [[unlikely]] {
      error = ServoMessageError::STATUS_ERROR;
      return;
    }

    servo_id = bytes[2];
    length = bytes[3];
    instruction_or_status = bytes[4];

    if constexpr (payload_size > 0)
      std::copy(bytes.begin() + header_size, bytes.begin() + (header_size + payload_size), payload.begin());

    checksum = bytes[total_size - 1];

    if (checksum != calculate_checksum()) [[unlikely]] {
      error = ServoMessageError::WRONG_CHECKSUM;
      return;
    }
  }

  [[nodiscard]] constexpr auto is_valid() const noexcept -> bool { return error == ServoMessageError::NONE; }

  [[nodiscard]] constexpr auto calculate_checksum() const noexcept -> std::uint8_t {
    std::uint32_t sum = servo_id + length + instruction_or_status;

    if constexpr (payload_size > 0)
      sum = std::accumulate(payload.begin(), payload.end(), sum);

    return common::as_byte(~sum);
  }

  [[nodiscard]] constexpr auto to_array() const noexcept -> std::array<std::uint8_t, total_size> {
    std::array<std::uint8_t, total_size> bytes{0xFF, 0xFF, servo_id, length, instruction_or_status};

    if constexpr (payload_size > 0)
      std::copy(payload.begin(), payload.end(), bytes.begin() + header_size);

    bytes[total_size - 1] = calculate_checksum();

    return bytes;
  }

  [[nodiscard]] constexpr auto operator==(ServoMessage const&) const noexcept -> bool = default;
};

}  // namespace device
