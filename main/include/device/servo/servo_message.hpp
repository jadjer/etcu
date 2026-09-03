//
// Created by jadjer on 4.09.26.
//

#pragma once

#include <array>
#include <numeric>

namespace device {

enum class ServoMode : std::uint8_t {
  PositionMode = 0x00,
  WheelMode = 0x01,
  PwmMode = 0x02,
  StepMode = 0x03,
};

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

template <std::size_t PayloadSize>
struct ServoMessage {
  std::uint8_t servo_id{};
  std::uint8_t length{static_cast<std::uint8_t>(PayloadSize + 2)};
  std::uint8_t instruction_or_status{};
  std::array<std::uint8_t, PayloadSize> payload{};
  std::uint8_t checksum{};

  ServoMessageError error{ServoMessageError::NONE};

  static constexpr std::size_t header_size{5};
  static constexpr std::size_t checksum_size{1};
  static constexpr std::size_t payload_size{PayloadSize};
  static constexpr std::size_t total_size{header_size + PayloadSize + checksum_size};

  constexpr ServoMessage() noexcept = default;

  constexpr explicit ServoMessage(std::uint8_t const servo_id, ServoInstruction const inst, std::array<std::uint8_t, PayloadSize> const& payload = {}) noexcept
      : servo_id{servo_id}, instruction_or_status{static_cast<std::uint8_t>(inst)}, payload{payload} {}

  constexpr explicit ServoMessage(std::uint8_t const expected_servo_id, std::array<std::uint8_t, total_size> const& bytes) noexcept {
    if (bytes[0] != 0xFF || bytes[1] != 0xFF) [[unlikely]] {
      error = ServoMessageError::WRONG_HEADER;
      return;
    }

    if (bytes[2] != expected_servo_id && expected_servo_id != 0xFE) [[unlikely]] {
      error = ServoMessageError::WRONG_ID;
      return;
    }

    if (bytes[3] != (payload_size + 2)) [[unlikely]] {
      error = ServoMessageError::WRONG_LENGTH;
      return;
    }

    if (bytes[4] != 0x00) [[unlikely]] {
      error = ServoMessageError::STATUS_ERROR;
      return;
    }

    servo_id = bytes[2];
    length = bytes[3];
    instruction_or_status = bytes[4];

    if constexpr (payload_size > 0) {
      std::copy(bytes.begin() + header_size, bytes.begin() + (header_size + payload_size), payload.begin());
    }

    checksum = bytes[total_size - 1];

    if (checksum != calculate_checksum()) [[unlikely]] {
      error = ServoMessageError::WRONG_CHECKSUM;
      return;
    }
  }

  [[nodiscard]] constexpr auto is_valid() const noexcept -> bool { return error == ServoMessageError::NONE; }

  [[nodiscard]] constexpr auto get_error() const noexcept -> ServoMessageError { return error; }

  [[nodiscard]] constexpr auto calculate_checksum() const noexcept -> std::uint8_t {
    std::uint32_t sum = servo_id + length + instruction_or_status;
    if constexpr (payload_size > 0) {
      sum = std::accumulate(payload.begin(), payload.end(), sum);
    }
    return static_cast<std::uint8_t>(~sum);
  }

  [[nodiscard]] constexpr auto to_array() const noexcept -> std::array<std::uint8_t, total_size> {
    std::array<std::uint8_t, total_size> bytes{};

    bytes[0] = 0xFF;
    bytes[1] = 0xFF;
    bytes[2] = servo_id;
    bytes[3] = length;
    bytes[4] = instruction_or_status;

    if constexpr (payload_size > 0) {
      std::copy(payload.begin(), payload.end(), bytes.begin() + header_size);
    }

    bytes[total_size - 1] = calculate_checksum();

    return bytes;
  }

  [[nodiscard]] constexpr auto operator==(ServoMessage const&) const noexcept -> bool = default;
};

}  // namespace device
