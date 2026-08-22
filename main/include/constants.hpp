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
inline constexpr type::Size MAX_BLE_PAYLOAD_SIZE{244};
inline constexpr type::String DEVICE_NAME{"ETCU"};
inline constexpr type::String SERVICE_UUID{"019fa351-08ac-76bf-b925-fe3ae2f765fb"};
inline constexpr type::String OTA_CHARACTERISTIC_UUID{"019fa351-08ac-7d45-8718-b4aa5af6756a"};
inline constexpr type::String CONTROL_CHARACTERISTIC_UUID{"019fa351-08ac-7309-804b-ad328e7c1ef1"};
inline constexpr type::String TELEMETRY_CHARACTERISTIC_UUID{"019fa351-08ac-7940-a519-6ef5087c0329"};
}  // namespace bluetooth

namespace pin {
inline constexpr type::GPIONum BUTTON_MODE{GPIO_NUM_0};
inline constexpr type::GPIONum SWITCH_BRAKE{GPIO_NUM_14};
inline constexpr type::GPIONum SWITCH_GUARD{GPIO_NUM_15};
inline constexpr type::GPIONum LED_MODE{GPIO_NUM_16};
inline constexpr type::GPIONum POWER_ENABLE{GPIO_NUM_18};
inline constexpr type::GPIONum ECU_WAKE_UP{GPIO_NUM_21};
}  // namespace pin

namespace uart {
namespace servo {
inline constexpr type::UARTPort PORT{UART_NUM_1};
inline constexpr type::GPIONum TX{GPIO_NUM_5};
inline constexpr type::GPIONum RX{GPIO_NUM_4};
inline constexpr type::Size BAUD_RATE{1'000'000};
}  // namespace servo
namespace ecu {
inline constexpr type::UARTPort PORT{UART_NUM_2};
inline constexpr type::GPIONum TX{GPIO_NUM_19};
inline constexpr type::GPIONum RX{GPIO_NUM_20};
inline constexpr type::Size BAUD_RATE{10'400};
}  // namespace ecu
}  // namespace uart

namespace adc {
inline constexpr type::ADCUnitId UNIT{ADC_UNIT_1};
inline constexpr type::ADCChannelId CHANNEL_A{ADC_CHANNEL_8};
inline constexpr type::ADCChannelId CHANNEL_B{ADC_CHANNEL_9};
}  // namespace adc

namespace system {
inline constexpr type::CoreId SYSTEM_CORE{0};
inline constexpr type::CoreId CRITICAL_CORE{1};
inline constexpr type::MilliSec SYSTEM_RATE{50};
inline constexpr type::MilliSec CRITICAL_RATE{50};
inline constexpr type::MilliSec LONG_PRESS{1500};
inline constexpr type::Position MISMATCH_THRESHOLD{static_cast<std::int32_t>(static_cast<float>(type::Position::MAX_VALUE) * 0.1f)};
inline constexpr type::Ticks DEBOUNCE_TICKS{3};
inline constexpr type::String NVS_NAMESPACE{"ETCU"};
}  // namespace system

}  // namespace constants
