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

#include "config/concepts.hpp"
#include "type/ecu.hpp"
#include "type/type.hpp"

namespace device {

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

  template <std::size_t PayloadSize>
  auto send_message(type::Message<PayloadSize> const& message) noexcept -> bool {
    static constexpr std::uint16_t echo_timeout_ms{100};
    static constexpr std::size_t packet_size{type::Message<PayloadSize>::total_size};

    // 1. Очищаем буфер UART перед отправкой
    if (!m_driver_uart.flush()) [[unlikely]] {
      ESP_LOGE("ECU", "Flush failed");
      return false;
    }

    std::array<std::uint8_t, packet_size> const packet = message.to_array();

    // 2. Отправляем данные (write возвращает int/ssize_t)
    int const written_bytes = m_driver_uart.write(packet);
    if (written_bytes < 0 || static_cast<std::size_t>(written_bytes) < packet_size) [[unlikely]] {
      // Исправлено: %zu для packet_size
      ESP_LOGE("ECU", "Written %d bytes from %zu", written_bytes, packet_size);
      return false;
    }

    // 3. Выделяем буфер под эхо-пакет правильного размера
    std::array<std::uint8_t, packet_size> echo_packet{};

    // 4. Исправлено: read возвращает bool. Если вернул false — значит, вычитали не всё или таймаут
    if (!m_driver_uart.read(echo_packet, echo_timeout_ms)) [[unlikely]] {
      ESP_LOGE("ECU", "Read echo bytes failed (Timeout or incomplete), expected %zu", packet_size);
      return false;
    }

    // 5. Исправлено: парсим эхо через std::expected, проверяя структуру и чексумму прямо из эфира
    std::expected<type::Message<PayloadSize>, type::MessageError> const expected_echo_message = type::Message<PayloadSize>::from_array(echo_packet);

    if (!expected_echo_message.has_value()) [[unlikely]] {
      ESP_LOGE("ECU", "Echo packet is corrupted (invalid checksum or layout)");
      return false;
    }

    // 6. Сравниваем исходное сообщение с эхо-сообщением через ваш оператор operator==
    if (message != expected_echo_message.value()) {
      ESP_LOGE("ECU", "Wrong echo: Sent data does not match received loopback data");
      return false;
    }

    return true;
  }

  template <std::size_t PayloadSize>
  auto receive_message(type::Message<PayloadSize>& message) noexcept -> bool {
    static constexpr std::uint16_t timeout_ms{1000};
    static constexpr std::size_t packet_size{type::Message<PayloadSize>::total_size};

    std::array<std::uint8_t, packet_size> packet{};

    // 1. Читаем данные. Драйвер UART сам проверяет, что вычитал строго packet_size байт
    if (!m_driver_uart.read(packet, timeout_ms)) [[unlikely]] {
      ESP_LOGE("ECU", "Read message failed (Timeout or Bus Error)");
      m_is_connected = false;
      return false;
    }

    // 2. Парсим пакет и получаем std::expected
    std::expected<type::Message<PayloadSize>, type::MessageError> const expected_message = type::Message<PayloadSize>::from_array(packet);

    // 3. Обрабатываем возможные ошибки парсинга
    if (!expected_message.has_value()) [[unlikely]] {
      type::MessageError const error = expected_message.error();

      if (error == type::MessageError::WRONG_LENGTH) {
        ESP_LOGE("ECU", "Receive failed: Packet has invalid length field");
      } else if (error == type::MessageError::WRONG_CHECKSUM) {
        ESP_LOGE("ECU", "Receive failed: Checksum mismatch");
      }

      return false;
    }

    // 4. Ошибок нет. Безопасно извлекаем и копируем сообщение в выходной параметр
    message = expected_message.value();  // или message = *parse_result;

    return true;
  }

  auto wake_up() noexcept -> bool {
    static constexpr type::Message<0> wakeup_message{.address = 0xFE, .length = 0x04, .mode = 0xFF, .payload = {}, .checksum = 0xFF};
    static constexpr type::Message<1> init_message{.address = 0x72, .length = 0x05, .mode = 0x00, .payload = {0xF0}, .checksum = 0x99};
    static constexpr type::Message<0> init_answer_packet{.address = 0x02, .length = 0x04, .mode = 0x00, .payload = {}, .checksum = 0xFA};

    m_driver_gpio.init();

    m_driver_gpio.disable();
    vTaskDelay(pdMS_TO_TICKS(70));

    m_driver_gpio.enable();
    vTaskDelay(pdMS_TO_TICKS(120));

    m_driver_uart.init();

    if (!send_message(wakeup_message)) {
      ESP_LOGE("ECU", "WAKE_UP Send failed");
      return false;
    }

    if (!send_message(init_message)) {
      ESP_LOGE("ECU", "INIT Send failed");
      return false;
    }

    type::Message<0> answer_message{};

    if (!receive_message(answer_message)) {
      ESP_LOGE("ECU", "INIT Receive failed");
      return false;
    }

    if (answer_message.to_array() != init_answer_packet.to_array()) {
      ESP_LOGE("ECU", "INIT Wrong answer message");
      return false;
    }

    return true;
  }

  auto connect() noexcept -> bool {
    if (m_is_connected) [[likely]]
      return true;

    if (!wake_up()) [[unlikely]]
      return false;

    m_is_connected = true;

    return true;
  }

  auto updateTable0() noexcept -> bool {
    static constexpr type::Message<1> request{.address = 0x72, .length = 5, .mode = 0x71, .payload = {}, .checksum = 0x18};

    if (!connect()) [[unlikely]]
      return false;

    if (!send_message(request)) [[unlikely]]
      return false;

    type::Message<11> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    return true;
  }

  auto updateTable10() noexcept -> bool {
    static constexpr type::Message<1> request{.address = 0x72, .length = 5, .mode = 0x71, .payload = {0x10}, .checksum = 0x8};

    if (!connect()) [[unlikely]]
      return false;

    if (!send_message(request)) [[unlikely]]
      return false;

    type::Message<18> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    return true;
  }

  auto updateTable11() noexcept -> bool {
    static constexpr type::Message<1> request{.address = 0x72, .length = 5, .mode = 0x71, .payload = {0x11}, .checksum = 0x7};

    if (!connect()) [[unlikely]]
      return false;

    if (!send_message(request)) [[unlikely]]
      return false;

    type::Message<21> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    return true;
  }

  auto updateTable20() noexcept -> bool {
    static constexpr type::Message<1> request{.address = 0x72, .length = 5, .mode = 0x71, .payload = {0x20}, .checksum = 0xF8};

    if (!connect()) [[unlikely]]
      return false;

    if (!send_message(request)) [[unlikely]]
      return false;

    type::Message<4> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    return true;
  }

  auto updateTable21() noexcept -> bool {
    static constexpr type::Message<1> request{.address = 0x72, .length = 5, .mode = 0x71, .payload = {0x21}, .checksum = 0xF7};

    if (!connect()) [[unlikely]]
      return false;

    if (!send_message(request)) [[unlikely]]
      return false;

    type::Message<7> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    return true;
  }

  auto updateTableD0() noexcept -> bool {
    static constexpr type::Message<1> request{.address = 0x72, .length = 5, .mode = 0x71, .payload = {0xD0}, .checksum = 0x48};

    if (!connect()) [[unlikely]]
      return false;

    if (!send_message(request)) [[unlikely]]
      return false;

    type::Message<15> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    return true;
  }

  auto updateTableD1() noexcept -> bool {
    static constexpr type::Message<1> request{.address = 0x72, .length = 5, .mode = 0x71, .payload = {0xD1}, .checksum = 0x47};

    if (!connect()) [[unlikely]]
      return false;

    if (!send_message(request)) [[unlikely]]
      return false;

    type::Message<7> response{};

    if (!receive_message(response)) [[unlikely]]
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
    if (!m_driver_uart.init()) [[unlikely]]
      return type::SystemError::ECUInitFault;

    if (!m_driver_gpio.init()) [[unlikely]]
      return type::SystemError::ECUInitFault;

    if (!connect()) [[unlikely]]
      return type::SystemError::ECUInitFault;

    return type::SystemError::None;
  }

  [[nodiscard]] auto update() noexcept -> type::SystemError {
    if (!connect()) [[unlikely]]
      return type::SystemError::ECUInitFault;

    // updateTable0();
    // updateTable10();
    // updateTable11();
    // updateTable20();
    // updateTable21();
    // updateTableD0();
    // updateTableD1();

    return type::SystemError::None;
  }

  [[nodiscard]] auto get_telemetry(type::ECUTelemetry& telemetry) noexcept -> type::SystemError {
    if (!connect()) [[unlikely]]
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
