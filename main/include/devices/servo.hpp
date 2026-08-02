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
#include <array>
#include "types.hpp"

namespace devices {

template <uart_port_t Port, gpio_num_t Tx, gpio_num_t Rx>
class Servo {
  static Position constexpr MinimalPosition = 0;
  static Position constexpr MaximalPosition = 100;

  auto calculateChecksum(Byte const* buffer,  std::size_t const length) const -> Byte {
        uint32_t sum = 0;
        for (size_t i = 0; i < length; ++i) {
            sum += buffer[i];
        }
        return static_cast<Byte>(~sum);
    }

  auto sendPacket(std::uint8_t id, ServoInstruction instruction,  std::uint8_t const* parameters, std::uint8_t param_len) const -> void {
        std::vector<std::uint8_t> packet;
        packet.reserve(6 + param_len);

        packet.push_back(0xFF);
        packet.push_back(0xFF);
        packet.push_back(id);
        packet.push_back(param_len + 2);
        packet.push_back(static_cast<Byte>(instruction));

        if (parameters && param_len > 0) {
            packet.insert(packet.end(), parameters, parameters + param_len);
        }

        uint8_t checksum = calculateChecksum(packet.data() + 2, packet.size() - 2);
        packet.push_back(checksum);

        uart_flush_input(Port);
        uart_write_bytes(Port, packet.data(), packet.size());
    }

  auto receivePacket(uint8_t id, uint8_t* out_payload, uint8_t expected_payload_len) -> bool {
        if (id == 0xFE) return false;

        size_t expected_total = 6 + expected_payload_len;
        std::vector<uint8_t> response(expected_total);

        int read_bytes = uart_read_bytes(Port, response.data(), expected_total, pdMS_TO_TICKS(30));
        if (read_bytes < static_cast<int>(expected_total)) {
            return false;
        }

        if (response[0] != 0xFF || response[1] != 0xFF || response[2] != id) {
            return false;
        }

        uint8_t calculated_cs = calculateChecksum(response.data() + 2, expected_total - 3);
        if (calculated_cs != response[expected_total - 1]) {
            // ESP_LOGE(LOG_TAG, "Checksum error from ID %d", id);
          // TODO
            return false;
        }

        if (response[4] != 0x00) {
            // ESP_LOGW(LOG_TAG, "Servo ID %d status error: 0x%02X", id, response[4]);
          // TODO
        }

        if (expected_payload_len > 0 && out_payload != nullptr) {
            std::memcpy(out_payload, response.data() + 5, expected_payload_len);
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
    if (auto const err = uart_param_config(Port, &config); err != ESP_OK) {
      return SystemError::ServoInitFault;
    }
    if (auto const err = uart_set_pin(Port, Tx, Rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); err != ESP_OK) {
      return SystemError::ServoInitFault;
    }
    if (auto const err = uart_driver_install(Port, 1024, 1024, 0, nullptr, 0); err != ESP_OK) {
      return SystemError::ServoInitFault;
    }

    return SystemError::None;
  }

  auto self_test() noexcept -> SystemError { return SystemError::None; }

  auto set_position(Position const target_position) noexcept -> SystemError {
    auto const target_steps = static_cast<std::uint16_t>((static_cast<std::uint32_t>(target_position) * 4095) / 10000);

    std::array<std::uint8_t, 7> const packet = {
        0xFF, 0xFF, 0x01, 0x03, 0x2A, static_cast<uint8_t>(target_steps >> 8), static_cast<uint8_t>(target_steps & 0xFF),
    };
    int const packet_size = packet.size();

    int const written = uart_write_bytes(Port, packet.data(), packet_size);
    if (written != packet_size) {
      return SystemError::ServoCommsFault;
    }

    return SystemError::None;
  }

  auto get_telemetry(ServoTelemetry& telemetry) noexcept -> SystemError {
    // Параметры запроса: стартовый адрес 0x38 (REG_PRESENT_POSITION), длина чтения 8 байт
    uint8_t params[2] = { static_cast<Byte>(ServoRegister::RegPresentPosition), 12 };
    sendPacket(1, ServoInstruction::InstRead, params, 2);

    uint8_t payload[8];
    // Ожидаем пакет ответа, содержащий 8 байт полезной нагрузки
    if (receivePacket(1, payload, 12)) {
      // Парсинг данных из формата Little-Endian
      telemetry.position = static_cast<int16_t>((payload[1] << 8) | payload[0]);
      // t.speed       = static_cast<int16_t>((payload[3] << 8) | payload[2]);
      // t.load        = static_cast<int16_t>((payload[5] << 8) | payload[4]);
      // t.voltage     = static_cast<float>(payload[6]) / 10.0f;
      telemetry.temperature = static_cast<int8_t>(payload[7]);

      int16_t raw_current = static_cast<int16_t>((payload[11] << 8) | payload[10]);

      telemetry.current     = (static_cast<float>(raw_current) * 6.5f) / 1000.0f;
    }

    return SystemError::None;
  }

   // Проверка связи (Ping)
  auto ping(uint8_t id) -> bool {
        sendPacket(id, ServoInstruction::InstPing, nullptr, 0);
        return receivePacket(id, nullptr, 0);
    }

    // Включение/выключение фиксации вала (Torque Enable)
    auto setTorque(uint8_t id, bool enable) -> bool {
        uint8_t params[2] = { static_cast<Byte>(ServoRegister::RegTorqueEnable), static_cast<Byte>(enable ? 1 : 0) };
        sendPacket(id, ServoInstruction::InstWrite, params, 2);
        return receivePacket(id, nullptr, 0);
    }

    // Установка позиции для одиночного сервопривода
    auto setPosition(uint8_t id, uint16_t position, uint16_t speed = 0, uint16_t time = 0) -> bool {
        uint8_t params[7];
        params[0] = static_cast<Byte>(ServoRegister::RegTargetPosition);
        params[1] = position & 0xFF;
        params[2] = (position >> 8) & 0xFF;
        params[3] = time & 0xFF;
        params[4] = (time >> 8) & 0xFF;
        params[5] = speed & 0xFF;
        params[6] = (speed >> 8) & 0xFF;

        sendPacket(id, ServoInstruction::InstWrite, params, 7);
        return receivePacket(id, nullptr, 0);
    }

    // Чтение текущей позиции вала (0 - 4095)
    auto readPosition(uint8_t id) -> int32_t {
        uint8_t params[2] = { static_cast<Byte>(ServoRegister::RegPresentPosition), 2 };
        sendPacket(id, ServoInstruction::InstRead, params, 2);

        uint8_t payload[2];
        if (receivePacket(id, payload, 2)) {
            return static_cast<int16_t>((payload[1] << 8) | payload[0]);
        }
        return -1;
    }

    // Чтение текущей скорости
    auto readSpeed(uint8_t id) -> int16_t {
        uint8_t params[2] = { static_cast<Byte>(ServoRegister::RegPresentSpeed), 2 };
        sendPacket(id, ServoInstruction::InstRead, params, 2);

        uint8_t payload[2];
        if (receivePacket(id, payload, 2)) {
            return static_cast<int16_t>((payload[1] << 8) | payload[0]);
        }
        return 0;
    }

    // Чтение напряжения питания (в Вольтах)
    auto readVoltage(uint8_t id) -> float {
        uint8_t params[2] = { static_cast<Byte>(ServoRegister::RegPresentVoltage), 1 };
        sendPacket(id, ServoInstruction::InstRead, params, 2);

        uint8_t payload = 0;
        if (receivePacket(id, &payload, 1)) {
            return static_cast<float>(payload) / 10.0f;
        }
        return -1.0f;
    }

    // Чтение температуры (в градусах Цельсия)
    auto readTemperature(uint8_t id) -> int8_t {
        uint8_t params[2] = { static_cast<Byte>(ServoRegister::RegPresentTemp), 1 };
        sendPacket(id, ServoInstruction::InstRead, params, 2);

        uint8_t payload = 0;
        if (receivePacket(id, &payload, 1)) {
            return static_cast<int8_t>(payload);
        }
        return -1;
    }
};

}  // namespace devices
