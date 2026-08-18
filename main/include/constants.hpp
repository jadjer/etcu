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
// Created by jadjer on 28.07.26.
//

#pragma once

#include <type.hpp>

namespace constants {

namespace bluetooth {
auto constexpr DEVICE_NAME = "ETCU";
auto constexpr SERVICE_UUID = "019fa351-08ac-76bf-b925-fe3ae2f765fb";
auto constexpr OTA_CHARACTERISTIC_UUID = "019fa351-08ac-7d45-8718-b4aa5af6756a";
auto constexpr CONTROL_CHARACTERISTIC_UUID = "019fa351-08ac-7309-804b-ad328e7c1ef1";
auto constexpr TELEMETRY_CHARACTERISTIC_UUID = "019fa351-08ac-7940-a519-6ef5087c0329";
}  // namespace bluetooth

namespace pin {
type::GPIONum constexpr BUTTON_MODE = GPIO_NUM_0;
type::GPIONum constexpr SWITCH_BRAKE = GPIO_NUM_14;
type::GPIONum constexpr SWITCH_GUARD = GPIO_NUM_15;
type::GPIONum constexpr LED_MODE = GPIO_NUM_16;
type::GPIONum constexpr LED_STATE = GPIO_NUM_17;
type::GPIONum constexpr POWER_ENABLE = GPIO_NUM_18;
}  // namespace pin

namespace uart {
namespace servo {
type::UARTPort constexpr PORT = UART_NUM_1;
type::GPIONum constexpr TX = GPIO_NUM_5;
type::GPIONum constexpr RX = GPIO_NUM_4;
}  // namespace servo
namespace ecu {
type::UARTPort constexpr PORT = UART_NUM_2;
type::GPIONum constexpr TX = GPIO_NUM_19;
type::GPIONum constexpr RX = GPIO_NUM_20;
}  // namespace ecu
}  // namespace uart

namespace adc {
type::ADCUnit constexpr UNIT = ADC_UNIT_1;
type::ADCChannel constexpr CHANNEL_A = ADC_CHANNEL_8;
type::ADCChannel constexpr CHANNEL_B = ADC_CHANNEL_9;
}  // namespace adc

namespace system {
type::Size constexpr MAX_BLE_PAYLOAD_SIZE = 244;
type::CoreId constexpr SYSTEM_CORE = 0;
type::CoreId constexpr CRITICAL_CORE = 1;
type::MilliSec constexpr SYSTEM_RATE = 50;
type::MilliSec constexpr CRITICAL_RATE = 50;
type::MilliSec constexpr LONG_PRESS = 1500;
type::Position constexpr MISMATCH_THRESHOLD = 10;
std::uint32_t constexpr DEBOUNCE_TICKS = 3;
}  // namespace system

}  // namespace constants
