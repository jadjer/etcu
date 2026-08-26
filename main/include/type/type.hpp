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

using Load = common::BoundedValue<primitive::Load, limit::load_min, limit::load_max>;
using Volt = common::BoundedValue<primitive::Volt, limit::volt_min, limit::volt_max>;
using MilliVolt = common::BoundedValue<primitive::MilliVolt, limit::milli_volt_min, limit::milli_volt_max>;
using Position = common::BoundedValue<primitive::Position, limit::position_min, limit::position_max>;
using ServoPosition = common::BoundedValue<primitive::ServoPosition, limit::servo_position_min, limit::servo_position_max>;
using RPM = common::BoundedValue<primitive::RPM, limit::rpm_min, limit::rpm_max>;
using Speed = common::BoundedValue<primitive::Speed, limit::speed_min, limit::speed_max>;
using Current = common::BoundedValue<primitive::Current, limit::current_min, limit::current_max>;
using Temperature = common::BoundedValue<primitive::Temperature, limit::temperature_min, limit::temperature_max>;
using MilliSec = common::BoundedValue<primitive::MilliSec, limit::milli_sec_min, limit::milli_sec_max>;

}  // namespace type
