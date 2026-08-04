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
// Created by jadjer on 23.07.26.
//

#pragma once

#include <driver/uart.h>

#include <utility>
#include "commons/map_range.hpp"
#include "types.hpp"

namespace devices {

template <uart_port_t port, gpio_num_t tx, gpio_num_t rx, ServoId servoId = 1>
class Servo {
  ServoCalibrationData m_calibration_data{
      .position_minimal = 600,
      .position_maximal = 1250,
  };

  template <std::size_t packetSize>
    requires(packetSize >= 6)
  [[nodiscard]] auto calculate_checksum_for_packet(std::array<Byte, packetSize> const bytes) const noexcept -> Byte {
    std::size_t constexpr start_index = 2;
    std::size_t constexpr end_index = packetSize - 1;

    std::uint32_t sum = 0;

    for (std::size_t i = start_index; i < end_index; ++i) {
      sum += bytes[i];
    }

    return static_cast<Byte>(~(sum & 0xFF));
  }

  template <std::size_t paramSize>
  auto send_packet(ServoInstruction const instruction, std::array<Byte, paramSize> const& parameters) const noexcept -> void {
    std::size_t constexpr packet_size = paramSize + 6;
    std::array<Byte, packet_size> packet;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servoId;
    packet[3] = (paramSize + 2);
    packet[4] = +instruction;

    if constexpr (paramSize > 0) {
      for (std::size_t i = 0; i < paramSize; ++i) {
        packet[5 + i] = parameters[i];
      }
    }

    packet[5 + paramSize] = calculate_checksum_for_packet(packet);

    uart_flush_input(port);
    uart_write_bytes(port, packet.data(), packet.size());
  }

  template <std::size_t paramSize>
  auto receive_packet(std::array<Byte, paramSize>& payload) noexcept -> bool {
    if (servoId == 0xFE)
      return false;

    std::size_t constexpr packetSize = paramSize + 6;

    std::array<Byte, packetSize> response;

    int const read_bytes = uart_read_bytes(port, response.data(), packetSize, pdMS_TO_TICKS(30));

    if (std::cmp_less(read_bytes, packetSize)) {
      return false;
    }

    if (response[0] != 0xFF || response[1] != 0xFF || response[2] != servoId) {
      return false;
    }

    Byte const calculated_check_sum = calculate_checksum_for_packet(response);

    if (calculated_check_sum != response[packetSize - 1]) {
      return false;
    }

    if constexpr (paramSize > 0) {
      for (std::size_t i = 0; i < paramSize; ++i) {
        payload[i] = static_cast<Byte>(response[5 + i]);
      }
    }

    return true;
  }

 public:
  auto init() noexcept -> SystemError {
    uart_config_t constexpr config = {
        .baud_rate = 1'000'000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 104,
        .source_clk = UART_SCLK_DEFAULT,
        .flags =
            {
                .allow_pd = false,
                .backup_before_sleep = false,
            },
    };
    if (auto const err = uart_param_config(port, &config); err != ESP_OK) {
      return SystemError::ServoInitError;
    }
    if (auto const err = uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); err != ESP_OK) {
      return SystemError::ServoInitError;
    }
    if (auto const err = uart_driver_install(port, 4096, 4096, 0, nullptr, 0); err != ESP_OK) {
      return SystemError::ServoInitError;
    }
    
    return SystemError::None;
  }

  auto set_calibrate(ServoCalibrationData const& calibration_data) noexcept -> void {
    m_calibration_data = calibration_data;

    set_position(m_calibration_data.position_minimal);
  }

  auto calibrate(ServoCalibrationData& calibration_data) noexcept -> void {
    set_mode(ServoMode::WheelMode);

    ServoPosition left_position, right_position;

    find_limit(-500, left_position);
    find_limit(500, right_position);

    set_mode(ServoMode::PositionMode);

    m_calibration_data.position_minimal = left_position + 50;
    m_calibration_data.position_maximal = right_position;

    calibration_data = m_calibration_data;
  }

  auto set_position(Position const target_position) noexcept -> void {
    Position constexpr minimalPosition = 0;
    Position constexpr maximalPosition = 100;

    ServoPosition const servo_position =
        commons::map_range(target_position, minimalPosition, maximalPosition, m_calibration_data.position_minimal, m_calibration_data.position_maximal);

    std::array<Byte, 7> params;
    params[0] = +ServoRegister::RegTargetPosition;
    params[1] = servo_position & 0xFF;
    params[2] = (servo_position >> 8) & 0xFF;
    params[3] = 0 & 0xFF;
    params[4] = (0 >> 8) & 0xFF;
    params[5] = 0 & 0xFF;
    params[6] = (0 >> 8) & 0xFF;
    send_packet(ServoInstruction::InstWrite, params);

    std::array<Byte, 0> payload;
    receive_packet(payload);
  }

  auto get_telemetry(ServoTelemetry& telemetry) noexcept -> bool {
    std::array<Byte, 2> constexpr params{+ServoRegister::RegPresentPosition, 15};
    send_packet(ServoInstruction::InstRead, params);

    std::array<Byte, 15> payload;
    if (receive_packet(payload)) {
      telemetry.is_connected = true;
      telemetry.position = static_cast<ServoPosition>(((payload[1] << 8) | payload[0]) & 0x7FFF);
      telemetry.speed = static_cast<Speed>(((payload[3] << 8) | payload[2]) & 0x7FFF);
      telemetry.load = static_cast<Load>(((payload[5] << 8) | payload[4]) & 0x03FF);
      telemetry.voltage = static_cast<Voltage>(payload[6]);
      telemetry.temperature = static_cast<Temperature>(payload[7]);
      telemetry.is_moved = payload[10];
      telemetry.current = static_cast<Current>(((payload[14] << 8) | payload[13]) & 0x7FFF);

      return true;
    }

    return false;
  }

  auto set_mode(ServoMode const mode) noexcept -> void {
    std::array const params{+ServoRegister::RegMode, +mode};
    send_packet(ServoInstruction::InstWrite, params);

    std::array<Byte, 0> payload;
    receive_packet(payload);
  }

  auto set_speed(std::int16_t const speed) noexcept -> void {
    auto const abs_speed = static_cast<Speed>(std::abs(speed));

    std::uint16_t reg_value = abs_speed;

    if (speed < 0) {
      reg_value |= 0x8000;
    }

    auto const low_byte = static_cast<Byte>(reg_value & 0xFF);
    auto const high_byte = static_cast<Byte>((reg_value >> 8) & 0xFF);

    std::array const params{+ServoRegister::RegTargetSpeed, low_byte, high_byte};
    send_packet(ServoInstruction::InstWrite, params);

    std::array<Byte, 0> payload;
    receive_packet(payload);
  }

  auto read_current(Current& current) noexcept -> bool {
    std::array<Byte, 2> constexpr params{+ServoRegister::RegPresentCurrent, 2};
    send_packet(ServoInstruction::InstRead, params);

    std::array<Byte, 2> payload;
    if (receive_packet(payload)) {
      current = static_cast<Current>(((payload[1] << 8) | payload[0]) & 0x7FFF);

      return true;
    }

    return false;
  }

  auto read_position(ServoPosition& position) noexcept -> bool {
    std::array<Byte, 2> constexpr params{+ServoRegister::RegPresentPosition, 2};
    send_packet(ServoInstruction::InstRead, params);

    std::array<Byte, 2> payload;

    if (receive_packet(payload)) {
      position = static_cast<ServoPosition>(payload[0] | (payload[1] << 8));

      return true;
    }

    return false;
  }

  auto find_limit(std::int16_t const speed, ServoPosition& position) -> bool {
    set_speed(speed);

    vTaskDelay(pdMS_TO_TICKS(200));

    ServoPosition last_position = 0;
    if (!read_position(last_position)) {
      set_speed(0);
      return false;
    }

    std::uint8_t constexpr confirmCounts = 10;  // 6 совпадений по 50мс = 300 мс полной остановки
    std::uint8_t stall_counter = 0;

    // Порог движения: если проехал меньше 2 тиков за 50 мс — считаем, что уперся
    ServoPosition constexpr min_movement_threshold = 2;

    while (true) {
      vTaskDelay(pdMS_TO_TICKS(50));

      ServoPosition current_position;
      if (!read_position(current_position)) {
        continue;  // Игнорируем единичные сбои шины датчика
      }

      // Вычисляем пройденное расстояние за 50 мс
      ServoPosition const delta = std::abs(current_position - last_position);

      if (delta < min_movement_threshold) {
        stall_counter++;

        if (stall_counter >= confirmCounts) {
          set_speed(0);  // Настоящий физический упор найден!
          position = current_position;
          return true;
        }
      } else {
        stall_counter = 0;  // Мотор уверенно крутится, сбрасываем счетчик остановки
      }

      // Запоминаем текущую точку для следующего шага через 50 мс
      last_position = current_position;
    }
  }
};

}  // namespace devices
