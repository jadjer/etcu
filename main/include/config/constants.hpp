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

#include <sdkconfig.h>

#include "driver/adc.hpp"
#include "type/type.hpp"

namespace constants {

namespace bluetooth {
inline constexpr std::size_t OTAPayloadSize{500};
inline constexpr std::string_view ServiceUUID{"019fa351-08ac-76bf-b925-fe3ae2f765fb"};
inline constexpr std::string_view OTACharUUID{"019fa351-08ac-7d45-8718-b4aa5af6756a"};
inline constexpr std::string_view ControlCharUUID{"019fa351-08ac-7309-804b-ad328e7c1ef1"};
inline constexpr std::string_view SysInfoCharUUID{"01a044f2-cf05-7494-aef5-a5298c878532"};
inline constexpr std::string_view WarningCharUUID{"01a0b03f-0dc8-7ad4-b376-769918cc91c2"};
inline constexpr std::string_view TelemetryCharUUID{"019fa351-08ac-7940-a519-6ef5087c0329"};
inline constexpr std::string_view CalibrationCharUUID{"01a07de1-71b0-730f-a4cc-d319b715b7e0"};
}  // namespace bluetooth

namespace pin {
inline constexpr int ButtonMode{CONFIG_ETCU_PIN_BUTTON_MODE};
inline constexpr int SwitchBrake{CONFIG_ETCU_PIN_SWITCH_BRAKE};
inline constexpr int SwitchGuard{CONFIG_ETCU_PIN_SWITCH_GUARD};
inline constexpr int LedMode{CONFIG_ETCU_PIN_LED_MODE};
inline constexpr int PowerEnable{CONFIG_ETCU_PIN_POWER_ENABLE};
}  // namespace pin

namespace uart {
namespace servo {
inline constexpr auto Port{CONFIG_ETCU_UART_SERVO_PORT};
inline constexpr int Tx{CONFIG_ETCU_UART_SERVO_TX};
inline constexpr int Rx{CONFIG_ETCU_UART_SERVO_RX};
inline constexpr std::size_t BaudRate{CONFIG_ETCU_UART_SERVO_BAUDRATE};
}  // namespace servo
namespace ecu {
inline constexpr auto Port{CONFIG_ETCU_UART_ECU_PORT};
inline constexpr int Tx{CONFIG_ETCU_UART_ECU_TX};
inline constexpr int Rx{CONFIG_ETCU_UART_ECU_RX};
inline constexpr std::size_t BaudRate{CONFIG_ETCU_UART_ECU_BAUDRATE};
}  // namespace ecu
}  // namespace uart

namespace adc {
inline constexpr auto Unit{CONFIG_ETCU_ADC_UNIT_NUM == 1 ? driver::ADCUnit::Unit1 : driver::ADCUnit::Unit2};
inline constexpr auto ChannelA{CONFIG_ETCU_ADC_CHANNEL_A};
inline constexpr auto ChannelB{CONFIG_ETCU_ADC_CHANNEL_B};
}  // namespace adc

namespace system {
inline constexpr std::string_view Name{"ETCU"};
inline constexpr type::Position MismatchThreshold(CONFIG_ETCU_ACCELERATOR_MISMATCH);

}  // namespace system

}  // namespace constants
