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

#include "common/bounded_value.hpp"
#include "type/limit.hpp"
#include "type/primitive.hpp"

namespace type {

namespace tag {

struct Position {};
struct MilliVolt {};
struct Volt {};
struct ServoPosition {};
struct RPM {};
struct Speed {};
struct Current {};
struct Temperature {};
struct MilliSec {};

}  // namespace tag

using Voltage = common::BoundedValue<primitive::Voltage, limit::VoltMin, limit::VoltMax, tag::Volt>;
using MilliVolt = common::BoundedValue<primitive::MilliVolt, limit::MilliVoltMin, limit::MilliVoltMax, tag::MilliVolt>;
using Position = common::BoundedValue<primitive::Position, limit::PositionMin, limit::PositionMax, tag::Position>;
using ServoPosition = common::BoundedValue<primitive::ServoPosition, limit::ServoPositionMin, limit::ServoPositionMax, tag::ServoPosition>;
using RPM = common::BoundedValue<primitive::RPM, limit::RpmMin, limit::RpmMax, tag::RPM>;
using Speed = common::BoundedValue<primitive::Speed, limit::SpeedMin, limit::SpeedMax, tag::Speed>;
using Current = common::BoundedValue<primitive::Current, limit::CurrentMin, limit::CurrentMax, tag::Current>;
using Temperature = common::BoundedValue<primitive::Temperature, limit::TemperatureMin, limit::TemperatureMax, tag::Temperature>;
using MilliSec = common::BoundedValue<primitive::MilliSec, limit::MilliSecMin, limit::MilliSecMax, tag::MilliSec>;

}  // namespace type
