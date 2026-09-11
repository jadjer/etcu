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

#include "type/primitive.hpp"

namespace type::limit {

static constexpr primitive::Load load_min{0};
static constexpr primitive::Load load_max{1000};

static constexpr primitive::Volt volt_min{0};
static constexpr primitive::Volt volt_max{100};

static constexpr primitive::AccPosition acc_pos_min{0};
static constexpr primitive::AccPosition acc_pos_max{3100};

static constexpr primitive::Position pos_min{0};
static constexpr primitive::Position pos_max{1000};

static constexpr primitive::ServoPosition servo_pos_min{0};
static constexpr primitive::ServoPosition servo_pos_max{4095};

static constexpr primitive::RPM rpm_min{0};
static constexpr primitive::RPM rpm_max{9000};

static constexpr primitive::Speed speed_min{0};
static constexpr primitive::Speed speed_max{200};

static constexpr primitive::Current current_min{0};
static constexpr primitive::Current current_max{2500};

static constexpr primitive::Temperature temperature_min{0};
static constexpr primitive::Temperature temperature_max{150};

static constexpr primitive::MilliSec milli_sec_min{0};
static constexpr primitive::MilliSec milli_sec_max{1000};

static constexpr primitive::Pressure pressure_min{0};
static constexpr primitive::Pressure pressure_max{100};

}  // namespace type::limit
