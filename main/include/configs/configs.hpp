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

#include "types.hpp"

namespace configs {

struct Pins {
  static constexpr GPIONum ModeButton = GPIO_NUM_0;
  static constexpr GPIONum Clutch = GPIO_NUM_2;
  static constexpr GPIONum Brake = GPIO_NUM_3;
  static constexpr GPIONum Guard = GPIO_NUM_4;
  static constexpr GPIONum ModeLed = GPIO_NUM_5;
  static constexpr GPIONum StatusLed = GPIO_NUM_6;
};

struct UART {
  struct Logger {
    static constexpr UARTPort Port = UART_NUM_0;
    static constexpr GPIONum Tx = GPIO_NUM_34;
    static constexpr GPIONum Rx = GPIO_NUM_35;
  };

  struct Servo {
    static constexpr UARTPort Port = UART_NUM_1;
    static constexpr GPIONum Tx = GPIO_NUM_17;
    static constexpr GPIONum Rx = GPIO_NUM_18;
  };

  struct ECU {
    static constexpr UARTPort Port = UART_NUM_2;
    static constexpr GPIONum Tx = GPIO_NUM_19;
    static constexpr GPIONum Rx = GPIO_NUM_20;
  };
};

struct ADC {
  static constexpr ADCUnit Unit = ADC_UNIT_1;
  static constexpr ADCChannel Hall1 = ADC_CHANNEL_3;
  static constexpr ADCChannel Hall2 = ADC_CHANNEL_6;
  static constexpr MilliVolt HallMismatchThreshold = 150;
};

struct System {
  static constexpr CoreID SystemCore = 0;
  static constexpr CoreID CriticalCore = 1;
  static constexpr MilliSec SystemRate = 50;
  static constexpr MilliSec CriticalRate = 50;
  static constexpr MilliSec LongPress = 1500;
};

}  // namespace configs
