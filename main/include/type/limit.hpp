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

inline constexpr primitive::Load load_min{0};
inline constexpr primitive::Load load_max{1000};

inline constexpr primitive::Volt volt_min{0};
inline constexpr primitive::Volt volt_max{100};

inline constexpr primitive::MilliVolt milli_volt_min{0};
inline constexpr primitive::MilliVolt milli_volt_max{3100};

inline constexpr primitive::Position position_min{0};
inline constexpr primitive::Position position_max{1000};

inline constexpr primitive::ServoPosition servo_position_min{0};
inline constexpr primitive::ServoPosition servo_position_max{4100};

inline constexpr primitive::RPM rpm_min{0};
inline constexpr primitive::RPM rpm_max{9000};

inline constexpr primitive::Speed speed_min{0};
inline constexpr primitive::Speed speed_max{200};

inline constexpr primitive::Current current_min{0};
inline constexpr primitive::Current current_max{2500};

inline constexpr primitive::Temperature temperature_min{0};
inline constexpr primitive::Temperature temperature_max{150};

inline constexpr primitive::MilliSec milli_sec_min{0};
inline constexpr primitive::MilliSec milli_sec_max{1000};

}  // namespace type::limits
