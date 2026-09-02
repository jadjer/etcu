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

#include <numeric>
#include <utility>

#include "common/convert.hpp"
#include "config/concepts.hpp"
#include "type/type.hpp"

namespace device {

// Структура ответа Keihin: [Addr][Len][Mode][Table] -> [PAYLOAD...] -> [CS]

template <std::size_t PayloadSize>
struct Message {
  static constexpr std::size_t header_size = 4;
  static constexpr std::size_t checksum_size = 1;
  static constexpr std::size_t total_size = header_size + PayloadSize + checksum_size;

  std::uint8_t address;
  std::uint8_t length;
  std::uint8_t mode;
  std::uint8_t table;
  std::array<std::uint8_t, PayloadSize> payload;
  std::uint8_t checksum;

  // Безопасное преобразование структуры в массив байт
  [[nodiscard]] auto to_array() const noexcept -> std::array<std::uint8_t, total_size> {
    std::array<std::uint8_t, total_size> bytes{};
    bytes[0] = address;
    bytes[1] = length;
    bytes[2] = mode;
    bytes[3] = table;
    if constexpr (PayloadSize > 0) {
      std::copy(payload.begin(), payload.end(), bytes.begin() + header_size);
    }
    bytes[total_size - 1] = checksum;
    return bytes;
  }

  static auto from_array(std::array<std::uint8_t, total_size> const& bytes) noexcept -> Message {
    Message msg{};
    msg.address = bytes[0];
    msg.length = bytes[1];
    msg.mode = bytes[2];
    msg.table = bytes[3];
    if constexpr (PayloadSize > 0) {
      std::copy(bytes.begin() + header_size, bytes.begin() + (header_size + PayloadSize), msg.payload.begin());
    }
    msg.checksum = bytes[total_size - 1];
    return msg;
  }
} __attribute__((packed));

struct EngineData {
  std::uint16_t rpm;
  std::uint16_t fuelInject;
  std::uint8_t ignitionAdvance;
  std::uint16_t tpsPercent;
  std::uint16_t tpsVolts;
  std::uint8_t ectTemp;
  std::uint16_t ectVolts;
  std::uint8_t iatTemp;
  std::uint16_t iatVolts;
  std::uint8_t mapPressure;
  std::uint16_t mapVolts;
  std::uint16_t batteryVolts;
  std::uint8_t speed;
  std::uint8_t state;
};

template <class DriverUart, class DriverGPIO>
  requires concepts::UART<DriverUart> && concepts::GPIO<DriverGPIO>
class ECU {
  DriverUart& m_driver_uart;
  DriverGPIO& m_driver_gpio;

  bool m_is_connected = false;

  EngineData m_engine_data{};

  template <std::size_t PacketSize>
  [[nodiscard]] auto calculate_checksum(std::array<std::uint8_t, PacketSize> const& bytes) const noexcept -> std::uint8_t {
    std::uint32_t const sum = std::accumulate(bytes.begin(), bytes.end() - 1, 0U);
    return static_cast<std::uint8_t>(0x100 - (sum & 0xFF));
  }

  template <std::size_t PacketSize>
  auto send_packet(std::array<std::uint8_t, PacketSize> const& packet) const noexcept -> bool {
    static constexpr std::uint16_t timeout_ms{150};

    m_driver_uart.flush();
    m_driver_uart.write(packet);

    std::array<std::uint8_t, PacketSize> echo_packet{};
    if (int const read_bytes = m_driver_uart.read(echo_packet, timeout_ms); std::cmp_less(read_bytes, PacketSize)) [[unlikely]] {
      ESP_LOGE("ECU", "Received wrong echo packet (%d bytes)", read_bytes);
      return false;
    }

    return true;
  }

  template <std::size_t PayloadSize>
  auto send_message(Message<PayloadSize> const& message) const noexcept -> bool {
    auto msg_copy = message;
    auto packet = msg_copy.to_array();

    packet.back() = calculate_checksum(packet);

    return send_packet(packet);
  }

  template <std::size_t PacketSize>
  auto receive_packet(std::array<std::uint8_t, PacketSize>& packet) noexcept -> bool {
    static constexpr std::uint16_t timeout_ms{150};

    if (int const read_bytes = m_driver_uart.read(packet, timeout_ms); std::cmp_less(read_bytes, PacketSize)) [[unlikely]] {
      ESP_LOGE("ECU", "Received %d bytes", read_bytes);
      return false;
    }

    return true;
  }

  template <std::size_t PayloadSize>
  auto receive_message(Message<PayloadSize>& message) noexcept -> bool {
    constexpr std::size_t package_size = Message<PayloadSize>::total_size;
    std::array<std::uint8_t, package_size> response_packet{};

    if (!receive_packet(response_packet)) {
      return false;
    }

    std::uint8_t const checksum_response = response_packet.back();
    std::uint8_t const checksum_calculated = calculate_checksum(response_packet);

    if (checksum_response != checksum_calculated) [[unlikely]] {
      ESP_LOGE("ECU", "Checksum %d != %d", checksum_response, checksum_calculated);
      return false;
    }

    message = Message<PayloadSize>::from_array(response_packet);

    return true;
  }

  auto wake_up() noexcept -> bool {
    static constexpr std::array<std::uint8_t, 4> wakeupPacket{0xFE, 0x04, 0x72, 0x8C};
    static constexpr std::array<std::uint8_t, 4> wakeupAnswerPacket{0x0E, 0x04, 0x72, 0x7C};

    if (!m_driver_uart.deinit()) {
      ESP_LOGE("ECU", "UART deinit");
      return false;
    }

    if (!m_driver_gpio.init()) {
      ESP_LOGE("ECU", "GPIO Init");
      return false;
    }

    if (!m_driver_gpio.disable()) {
      ESP_LOGE("ECU", "TX low");
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(70));

    if (!m_driver_gpio.enable()) {
      ESP_LOGE("ECU", "TX high");
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(120));

    if (!m_driver_uart.init()) {
      ESP_LOGE("ECU", "UART init");
      return false;
    }

    if (!send_packet(wakeupPacket)) {
      ESP_LOGE("ECU", "WAPE_UP send");
      return false;
    }

    std::array<std::uint8_t, 4> answerMessage{};

    if (!receive_packet(answerMessage)) {
      ESP_LOGE("ECU", "WAPE_UP receive");
      return false;
    }

    if (answerMessage != wakeupAnswerPacket) {
      ESP_LOGE("ECU", "WAPE_UP compare");
      return false;
    }

    return true;
  }

  auto initialize() noexcept -> bool {
    static constexpr std::array<std::uint8_t, 5> initPacket{0x72, 0x05, 0x00, 0xF0, 0x99};
    static constexpr std::array<std::uint8_t, 4> initAnswerPacket{0x02, 0x04, 0x00, 0xFA};

    if (!send_packet(initPacket)) {
      ESP_LOGE("ECU", "INIT send");
      return false;
    }

    std::array<std::uint8_t, 4> answerMessage{};

    if (!receive_packet(answerMessage)) {
      ESP_LOGE("ECU", "INIT receive");
      return false;
    }

    if (answerMessage != initAnswerPacket) {
      ESP_LOGE("ECU", "INIT compare");
      return false;
    }

    return true;
  }

  auto connect() noexcept -> bool {
    if (m_is_connected)
      return true;

    if (!wake_up())
      return false;

    if (!initialize())
      return false;

    m_is_connected = true;

    return true;
  }

  auto updateTable0() noexcept -> bool {
    static constexpr Message<0> request{.address = 0x72, .length = 5, .mode = 0x71, .table = 0x0, .payload = {}, .checksum = 0x18};
    if (!send_message(request))
      return false;

    Message<10> response{};

    if (!receive_message(response))
      return false;

    return true;
  }

  auto updateTable10() noexcept -> bool {
    static constexpr Message<0> request{.address = 0x72, .length = 5, .mode = 0x71, .table = 0x10, .payload = {}, .checksum = 0x8};
    if (!send_message(request))
      return false;

    Message<17> response{};

    if (!receive_message(response))
      return false;

    return true;
  }

  auto updateTable11() noexcept -> bool {
    static constexpr Message<0> request{.address = 0x72, .length = 5, .mode = 0x71, .table = 0x11, .payload = {}, .checksum = 0x7};
    if (!send_message(request))
      return false;

    Message<20> response{};

    if (!receive_message(response))
      return false;

    return true;
  }

  auto updateTable20() noexcept -> bool {
    static constexpr Message<0> request{.address = 0x72, .length = 5, .mode = 0x71, .table = 0x20, .payload = {}, .checksum = 0xF8};
    if (!send_message(request))
      return false;

    Message<3> response{};

    if (!receive_message(response))
      return false;

    return true;
  }

  auto updateTable21() noexcept -> bool {
    static constexpr Message<0> request{.address = 0x72, .length = 5, .mode = 0x71, .table = 0x21, .payload = {}, .checksum = 0xF7};
    if (!send_message(request))
      return false;

    Message<6> response{};

    if (!receive_message(response))
      return false;

    return true;
  }

  auto updateTableD0() noexcept -> bool {
    static constexpr Message<0> request{.address = 0x72, .length = 5, .mode = 0x71, .table = 0xD0, .payload = {}, .checksum = 0x48};
    if (!send_message(request))
      return false;

    Message<14> response{};

    if (!receive_message(response))
      return false;

    return true;
  }

  auto updateTableD1() noexcept -> bool {
    static constexpr Message<0> request{.address = 0x72, .length = 5, .mode = 0x71, .table = 0xD1, .payload = {}, .checksum = 0x47};
    if (!send_message(request))
      return false;

    Message<6> response{};

    if (!receive_message(response))
      return false;

    return true;
  }

 public:
  constexpr explicit ECU(DriverUart& driver_uart, DriverGPIO& driver_gpio) noexcept : m_driver_uart{driver_uart}, m_driver_gpio{driver_gpio} {}

  constexpr ECU() noexcept = delete;

  ECU(ECU const&) noexcept = delete;
  auto operator=(ECU const&) noexcept -> ECU& = delete;

  ECU(ECU&&) noexcept = delete;
  auto operator=(ECU&&) noexcept -> ECU& = delete;

  constexpr ~ECU() noexcept = default;

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_driver_uart.init())
      return type::SystemError::ECUInitFault;

    if (!m_driver_gpio.init())
      return type::SystemError::ECUInitFault;

    if (!connect())
      return type::SystemError::ECUInitFault;

    return type::SystemError::None;
  }

  [[nodiscard]] auto update() noexcept -> type::SystemError {
    if (!connect())
      return type::SystemError::ECUInitFault;

    updateTable0();
    updateTable10();
    updateTable11();
    updateTable20();
    updateTable21();
    updateTableD0();
    updateTableD1();

    return type::SystemError::None;
  }

  [[nodiscard]] auto get_telemetry(type::ECUTelemetry& telemetry) noexcept -> type::SystemError {
    if (!connect())
      return type::SystemError::ECUInitFault;

    telemetry.is_connected = m_is_connected;
    telemetry.rpm = type::RPM{m_engine_data.rpm};
    telemetry.speed = type::Speed{m_engine_data.speed};
    telemetry.tps = type::Position{m_engine_data.tpsPercent};
    telemetry.is_started = false;
    telemetry.is_clutch_enabled = false;

    return type::SystemError::None;
  }
};

}  // namespace device
