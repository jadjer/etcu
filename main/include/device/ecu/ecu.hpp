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

#include "config/concepts.hpp"
#include "type/type.hpp"

namespace device {

// struct EngineData {
//   std::uint16_t rpm;
//   std::uint16_t fuelInject;
//   std::uint8_t ignitionAdvance;
//   std::uint8_t unkData1;
//   std::uint8_t unkData2;
//   std::uint8_t unkData3;
// } __attribute__((packed));
//
// struct ErrorData {};
//
// struct SensorsData {
//   float tpsPercent;
//   float tpsVolts;
//   std::uint8_t ectTemp;
//   float ectVolts;
//   std::uint8_t iatTemp;
//   float iatVolts;
//   std::uint8_t mapPressure;
//   float mapVolts;
// } __attribute__((packed));
//
// struct UnknownData {
//   std::uint8_t unkData1;
//   std::uint8_t unkData2;
//   std::uint8_t unkData3;
//   std::uint8_t unkData4;
//   std::uint8_t unkData5;
//   std::uint8_t unkData6;
//   std::uint8_t unkData7;
//   std::uint8_t unkData8;
//   std::uint8_t unkData9;
//   std::uint8_t unkData10;
//   std::uint8_t unkData11;
//   std::uint8_t unkData12;
//   std::uint8_t unkData13;
//   std::uint8_t unkData14;
//   std::uint8_t unkData15;
//   std::uint8_t unkData16;
//   std::uint8_t unkData17;
//   std::uint8_t unkData18;
//   std::uint8_t unkData19;
//   std::uint8_t unkData20;
//   std::uint8_t unkData21;
//   std::uint8_t unkData22;
//   std::uint8_t unkData23;
//   std::uint8_t unkData24;
//   std::uint8_t unkData25;
//   std::uint8_t unkData26;
//   std::uint8_t unkData27;
//   std::uint8_t unkData28;
//   std::uint8_t unkData29;
//   std::uint8_t unkData30;
//   std::uint8_t unkData31;
//   std::uint8_t unkData32;
//   std::uint8_t unkData33;
//   std::uint8_t unkData34;
//   std::uint8_t unkData35;
//   std::uint8_t unkData36;
//   std::uint8_t unkData37;
//   std::uint8_t unkData38;
//   std::uint8_t unkData39;
//   std::uint8_t unkData40;
// } __attribute__((packed));
//
// struct VehicleData {
//   std::string id;
//   float batteryVolts;
//   std::uint8_t speed;
//   std::uint8_t state;
// } __attribute__((packed));
//
// struct CommandResult {
//   std::uint8_t code;
//   std::uint8_t command;
//   std::uint8_t length;
//   std::uint8_t checksum;
//   std::uint8_t* data;
// } __attribute__((packed));

template <class DriverUart, class DriverGPIO>
  requires concepts::UART<DriverUart> && concepts::GPIO<DriverGPIO>

class ECU {
  DriverUart& m_driver_uart;
  DriverGPIO& m_driver_gpio;

 public:
  constexpr explicit ECU(DriverUart& driver_uart, DriverGPIO& driver_gpio) noexcept : m_driver_uart{driver_uart}, m_driver_gpio{driver_gpio} {}

  constexpr ECU() noexcept = delete;

  ECU(ECU const&) noexcept = delete;
  auto operator=(ECU const&) noexcept -> ECU& = delete;

  ECU(ECU&&) noexcept = delete;
  auto operator=(ECU&&) noexcept -> ECU& = delete;

  constexpr ~ECU() noexcept = default;

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    m_driver_uart.deinit();
    m_driver_gpio.init();

    m_driver_gpio.enable();
    vTaskDelay(pdMS_TO_TICKS(300));

    m_driver_gpio.disable();
    vTaskDelay(pdMS_TO_TICKS(25));

    m_driver_gpio.enable();
    vTaskDelay(pdMS_TO_TICKS(25));

    return type::SystemError::None;
  }

  [[nodiscard]] auto update() noexcept -> type::SystemError { return type::SystemError::None; }  // NOLINT

  [[nodiscard]] auto get_telemetry(type::ECUTelemetry& telemetry) const noexcept -> type::SystemError {  // NOLINT
    std::ignore = m_driver_gpio.enable();

    telemetry.is_connected = false;
    telemetry.rpm = type::RPM{0};
    telemetry.speed = type::Speed{0};
    telemetry.tps = type::Position{0};
    telemetry.is_started = false;
    telemetry.is_clutch_enabled = false;

    return type::SystemError::None;
  }
};

}  // namespace device
