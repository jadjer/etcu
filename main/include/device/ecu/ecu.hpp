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

#include <span>
#include "common/calculate.hpp"
#include "config/concepts.hpp"
#include "device/ecu/ecu_message.hpp"
#include "type/type.hpp"

namespace device {

enum class EngineState : std::uint8_t {
  GEAR_ON = 0x00,
  NEUTRAL = 0x01,
  SIDE_STAND_AND_GEAR_ON = 0x02,
  SIDE_STAND = 0x03,
};

struct EngineData {
  bool is_running;
  EngineState state;
  std::uint8_t speed;
  std::uint8_t ect_temp;
  std::uint8_t iat_temp;
  std::uint8_t ect_voltage;
  std::uint8_t iat_voltage;
  std::uint8_t tps_percent;
  std::uint8_t tps_voltage;
  std::uint8_t map_voltage;
  std::uint8_t map_pressure;
  std::uint8_t battery_voltage;
  std::uint8_t ignition_advance;
  std::uint16_t rpm;
  std::uint16_t fuel_inject;

  bool mil_state;               // Статус лампы Check Engine
  std::uint8_t dtc_count;        // Количество активных ошибок
  std::uint16_t dtc_code_1;      // Код первой ошибки (например, P0105 -> 0x0105)
  std::uint16_t dtc_code_2;      // Код второй ошибки

  // Топливные коррекции (0x6x)
  std::uint8_t short_term_trim;  // Краткосрочная коррекция топлива (-128%...+127% или в попугаях)
  std::uint8_t long_term_trim;   // Долгосрочная коррекция топлива
  std::uint16_t coil_dwell_time;// Время заряда катушки зажигания (обычно в мкс)

  // Экология и Периферия (0x7x)
  std::uint16_t o2_voltage;     // Напряжение датчика кислорода (мВ)
  std::uint8_t iacv_steps;      // Положение клапана холостого хода (шаги или %)
  std::uint8_t lean_angle_volt; // Напряжение датчика падения/наклона
};

template <class DriverUart, class DriverGPIO>
  requires concepts::UART<DriverUart> && concepts::GPIO<DriverGPIO>
class ECU {
  static constexpr std::size_t header_size{2};
  static constexpr std::size_t payload_size{20};
  static constexpr std::uint16_t timeout_ms{100};
  static constexpr std::array m_supported_tables{0x0, 0x10, 0x11, 0x20, 0x21, 0x60, 0x61, 0x70, 0x71, 0xD0, 0xD1};

  DriverUart& m_driver_uart;
  DriverGPIO& m_driver_gpio;

  bool m_is_connected = false;
  EngineData m_engine_data{};
  std::span<std::uint8_t const> m_active_tables{};
  std::array<std::uint8_t, m_supported_tables.size()> m_active_tables_buffer{};

  template <std::size_t PayloadSize>
  auto send_message(ECUMessage<PayloadSize> const& message) noexcept -> bool {
    static constexpr std::size_t packet_size{ECUMessage<PayloadSize>::total_size};

    if (!m_driver_uart.flush()) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    if (int const written_bytes = m_driver_uart.write(message.to_array()); std::cmp_less(written_bytes, packet_size)) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    std::array<std::uint8_t, packet_size> echo_packet{};

    if (!m_driver_uart.read(echo_packet, timeout_ms)) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    auto const echo_message = ECUMessage<PayloadSize>{echo_packet};

    if (!echo_message.is_valid()) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    if (message != echo_message) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    return true;
  }

  template <std::size_t PayloadSize>
  auto receive_message(ECUMessage<PayloadSize>& message) noexcept -> bool {
    static constexpr std::size_t packet_size{ECUMessage<PayloadSize>::total_size};

    std::array<std::uint8_t, packet_size> received_packet{};

    if (!m_driver_uart.read(received_packet, timeout_ms)) [[unlikely]] {
      m_is_connected = false;
      return false;
    }

    auto const received_message = ECUMessage<PayloadSize>{received_packet};

    if (!received_message.is_valid()) [[unlikely]] {
      return false;
    }

    message = received_message;

    return true;
  }

  auto wake_up() noexcept -> bool {
    static constexpr ECUMessage wakeup_message{0xFE, ECUMode::WAKE_UP};
    static constexpr std::array init_payload{common::as_byte(0xF0)};
    static constexpr ECUMessage init_message{0x72, ECUMode::INIT, init_payload};
    static constexpr ECUMessage init_answer_packet{0x02, ECUMode::INIT};

    if (!m_driver_gpio.init()) [[unlikely]]
      return false;

    if (!m_driver_gpio.disable()) [[unlikely]]
      return false;

    vTaskDelay(pdMS_TO_TICKS(70));

    if (!m_driver_gpio.enable()) [[unlikely]]
      return false;

    vTaskDelay(pdMS_TO_TICKS(120));

    if (!m_driver_uart.init()) [[unlikely]]
      return false;

    if (!send_message(wakeup_message)) [[unlikely]]
      return false;

    vTaskDelay(pdMS_TO_TICKS(200));

    if (!send_message(init_message)) [[unlikely]]
      return false;

    ECUMessage answer_message{};

    if (!receive_message(answer_message)) [[unlikely]]
      return false;

    if (answer_message != init_answer_packet) [[unlikely]]
      return false;

    return true;
  }

  auto detect_tables() noexcept -> void {
    static constexpr std::size_t probe_size{1};
    static constexpr std::uint8_t address{0x72};
    static constexpr std::uint8_t payload_offset{0x0};

    std::size_t found_count = 0;

    for (std::uint8_t const table : m_supported_tables) {
      std::array const payload{table, payload_offset, common::as_byte(probe_size)};

      if (ECUMessage const request{address, ECUMode::READ_RANGE, payload}; !send_message(request)) {
        continue;
      }

      if (ECUMessage<header_size + probe_size> response{}; receive_message(response)) {
        m_active_tables_buffer[found_count++] = table;
      }
    }

    m_active_tables = std::span<std::uint8_t const>{m_active_tables_buffer.data(), found_count};
  }

  auto connect() noexcept -> bool {
    if (m_is_connected) [[likely]]
      return true;

    if (!wake_up()) [[unlikely]]
      return false;

    m_is_connected = true;

    detect_tables();

    return true;
  }

  auto parse_table_data(std::uint8_t const table, std::array<std::uint8_t, header_size + payload_size> const& payload) noexcept -> void {
    switch (table) {
      case 0x0:
        ESP_LOG_BUFFER_HEX("ECU ID", payload.data() + header_size, payload_size);

        break;

      case 0x10:
      case 0x11:
        ESP_LOG_BUFFER_HEX("ECU 1x", payload.data() + header_size, payload_size);

        m_engine_data.rpm = common::as_ulong(payload[header_size + 0], payload[header_size + 1]);
        m_engine_data.tps_voltage = common::calculateValueDivide256(payload[header_size + 2]);
        m_engine_data.tps_percent = common::calculateValueDivide16(payload[header_size + 3]);
        m_engine_data.ect_voltage = common::calculateValueDivide256(payload[header_size + 4]);
        m_engine_data.ect_temp = common::calculateValueMinus40(payload[header_size + 5]);
        m_engine_data.iat_voltage = common::calculateValueDivide256(payload[header_size + 6]);
        m_engine_data.iat_temp = common::calculateValueMinus40(payload[header_size + 7]);
        m_engine_data.map_voltage = common::calculateValueDivide256(payload[header_size + 8]);
        m_engine_data.map_pressure = payload[header_size + 9];
        m_engine_data.battery_voltage = common::calculateValueDivide10(payload[header_size + 12]);
        m_engine_data.speed = payload[header_size + 13];
        m_engine_data.fuel_inject = common::as_ulong(payload[header_size + 14], payload[header_size + 15]);
        m_engine_data.ignition_advance = payload[header_size + 16];

        break;

      case 0x20:
      case 0x21:
        ESP_LOG_BUFFER_HEX("ECU 2x", payload.data() + header_size, payload_size);

        m_engine_data.mil_state = (payload[header_size + 0] & 0x80) != 0;
        m_engine_data.dtc_count = payload[header_size + 1];
        m_engine_data.dtc_code_1 = common::as_ulong(payload[header_size + 2], payload[header_size + 3]);

        break;

      case 0x60:
      case 0x61:
        ESP_LOG_BUFFER_HEX("ECU 6x", payload.data() + header_size, payload_size);

        m_engine_data.short_term_trim = common::calculateValueDivide16(payload[header_size + 0]);
        m_engine_data.long_term_trim  = common::calculateValueDivide16(payload[header_size + 1]);
        m_engine_data.coil_dwell_time = common::calculateValueMultiply10(payload[header_size + 2]);

        break;

      case 0x70:
      case 0x71:
        ESP_LOG_BUFFER_HEX("ECU 7x", payload.data() + header_size, payload_size);

        m_engine_data.o2_voltage = common::calculateValueDivide256(payload[header_size + 0]);
        m_engine_data.iacv_steps = common::calculateValueDivide10(payload[header_size + 2]);

        // Дополнительный температурный датчик (например, температура масла/выхлопа), если он есть в этой таблице
        // m_engine_data.aux_temp = common::calculateValueMinus40(payload[header_size + 3]);

        break;

      case 0xD0:
        ESP_LOG_BUFFER_HEX("ECU D0", payload.data() + header_size, payload_size);

        break;

      case 0xD1:
        ESP_LOG_BUFFER_HEX("ECU D1", payload.data() + header_size, payload_size);

        m_engine_data.state = static_cast<EngineState>(payload[2]);
        m_engine_data.is_running = common::as_byte(payload[6]);

        break;

      default:
        ESP_LOGW("ECU", "Unknown active table parsing: 0x%02X", table);

        break;
    }
  }

  auto update_table_generic(std::uint8_t const table) noexcept -> bool {
    static constexpr std::uint8_t address{0x72};
    static constexpr std::uint8_t payload_offset{0};

    std::array const payload{table, payload_offset, common::as_byte(payload_size)};
    ECUMessage const request{address, ECUMode::READ_RANGE, payload};

    if (!send_message(request)) [[unlikely]]
      return false;

    ECUMessage<header_size + payload_size> response{};

    if (!receive_message(response)) [[unlikely]]
      return false;

    parse_table_data(table, response.payload);

    return true;
  }

  auto update_active_tables() noexcept -> bool {
    for (std::uint8_t const table : m_active_tables) {
      if (!update_table_generic(table)) [[unlikely]] {
        m_is_connected = false;
        return false;
      }
    }

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

    if (!update_active_tables()) [[unlikely]]
      return type::SystemError::ECUReadFault;

    return type::SystemError::None;
  }

  [[nodiscard]] auto get_telemetry(type::ECUTelemetry& telemetry) noexcept -> type::SystemError {
    if (!connect()) [[unlikely]]
      return type::SystemError::ECUInitFault;

    telemetry.is_connected = m_is_connected;
    telemetry.is_started = m_engine_data.is_running;
    telemetry.is_clutch_enabled = m_engine_data.state == EngineState::NEUTRAL;
    telemetry.rpm = type::RPM{m_engine_data.rpm};
    telemetry.battery = type::Volt{m_engine_data.battery_voltage};
    telemetry.speed = type::Speed{m_engine_data.speed};
    telemetry.map = type::Pressure{m_engine_data.map_pressure};
    telemetry.tps = type::Position{m_engine_data.tps_percent};
    telemetry.air = type::Temperature{m_engine_data.iat_temp};
    telemetry.coolant = type::Temperature{m_engine_data.ect_temp};

    return type::SystemError::None;
  }
};

}  // namespace device
