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

#include "type/type.hpp"
#include "driver/adc.hpp"
#include "driver/uart.hpp"

namespace constants {

namespace bluetooth {
inline constexpr std::size_t MaxBlePayloadSize{244};
inline constexpr std::string_view DeviceName{"ETCU"};
inline constexpr std::string_view ServiceUUID{"019fa351-08ac-76bf-b925-fe3ae2f765fb"};
inline constexpr std::string_view OTACharacteristicUUID{"019fa351-08ac-7d45-8718-b4aa5af6756a"};
inline constexpr std::string_view ControlCharacteristicUUID{"019fa351-08ac-7309-804b-ad328e7c1ef1"};
inline constexpr std::string_view TelemetryCharacteristicUUID{"019fa351-08ac-7940-a519-6ef5087c0329"};
}  // namespace bluetooth

namespace pin {
inline constexpr int ButtonMode{0};
inline constexpr int SwitchBrake{14};
inline constexpr int SwitchGuard{15};
inline constexpr int LedMode{16};
inline constexpr int PowerEnable{18};
inline constexpr int ECUWakeUp{21};
}  // namespace pin

namespace uart {
namespace servo {
inline constexpr auto Port{driver::UartPort::Uart1};
inline constexpr int Tx{5};
inline constexpr int Rx{4};
inline constexpr std::size_t BaudRate{1'000'000};
}  // namespace servo
namespace ecu {
inline constexpr auto Port{driver::UartPort::Uart2};
inline constexpr int Tx{19};
inline constexpr int Rx{20};
inline constexpr std::size_t BaudRate{10'400};
}  // namespace ecu
}  // namespace uart

namespace adc {
inline constexpr auto Unit{driver::ADCUnit::Unit1};
inline constexpr int ChannelA{8};
inline constexpr int ChannelB{9};
}  // namespace adc

namespace system {
inline constexpr std::uint8_t SystemCore{0};
inline constexpr std::uint8_t CriticalCore{1};
inline constexpr type::MilliSec SystemRate{50};
inline constexpr type::MilliSec CriticalRate{50};
inline constexpr type::MilliSec LongPressUS{1500};
inline constexpr type::Position MismatchThreshold(static_cast<std::int64_t>(type::Position::max_value) / 10);
inline constexpr std::uint16_t DebounceTicksUS{3};
inline constexpr std::string_view NVSNamespace{"ETCU"};
}  // namespace system

}  // namespace constants
