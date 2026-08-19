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

#include "constants.hpp"
#include "type.hpp"

#include "driver/adc.hpp"
#include "driver/gpio.hpp"
#include "driver/uart.hpp"

#include "device/accelerator.hpp"
#include "device/button.hpp"
#include "device/ecu/ecu.hpp"
#include "device/indicator.hpp"
#include "device/servo.hpp"
#include "device/switch.hpp"

namespace config {

namespace driver {
using ADCChannelA = ::driver::ADC<constants::adc::UNIT, constants::adc::CHANNEL_A>;
using ADCChannelB = ::driver::ADC<constants::adc::UNIT, constants::adc::CHANNEL_B>;
using UARTServo = ::driver::UART<constants::uart::servo::PORT, constants::uart::servo::TX, constants::uart::servo::RX>;
using PowerEnable = ::driver::GPIO<constants::pin::POWER_ENABLE, GPIO_MODE_OUTPUT>;
using UARTEcu = ::driver::UART<constants::uart::ecu::PORT, constants::uart::ecu::TX, constants::uart::ecu::RX>;
using ButtonMode = ::driver::GPIO<constants::pin::BUTTON_MODE, GPIO_MODE_INPUT, true>;
using LedMode = ::driver::GPIO<constants::pin::LED_MODE, GPIO_MODE_OUTPUT>;
using SwitchBrake = ::driver::GPIO<constants::pin::SWITCH_BRAKE, GPIO_MODE_INPUT>;
using SwitchGuard = ::driver::GPIO<constants::pin::SWITCH_GUARD, GPIO_MODE_INPUT>;
}  // namespace driver

namespace device {
using Accelerator = ::device::Accelerator<driver::ADCChannelA, driver::ADCChannelB, constants::system::MISMATCH_THRESHOLD>;
using Servo = ::device::Servo<driver::UARTServo, driver::PowerEnable>;
using ECU = ::device::ECU<driver::UARTEcu>;
using ButtonMode = ::device::Button<driver::ButtonMode>;
using LedMode = ::device::Indicator<driver::LedMode>;
using Brake = ::device::Switch<driver::SwitchBrake>;
using Guard = ::device::Switch<driver::SwitchGuard>;

}  // namespace device

}  // namespace config
