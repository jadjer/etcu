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
// Created by jadjer on 24.08.26.
//

#pragma once

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_adc/adc_oneshot.h>
#include <array>
#include <cstdint>
#include <string_view>

namespace type::primitive {

using Load = std::uint16_t;
using Time = std::uint16_t;
using Volt = std::uint8_t;
using MilliVolt = std::uint16_t;
using Position = std::uint16_t;
using ServoPosition = std::uint16_t;
using ServoSpeed = std::uint16_t;
using RPM = std::uint16_t;
using Speed = std::uint8_t;
using Current = std::uint16_t;
using Temperature = std::uint8_t;
using MilliSec = std::uint16_t;
using FixedString = std::array<char, 16>;

}  // namespace type::primitive
