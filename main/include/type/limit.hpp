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

inline constexpr primitive::Voltage VoltMin{0};
inline constexpr primitive::Voltage VoltMax{100};

inline constexpr primitive::MilliVolt MilliVoltMin{0};
inline constexpr primitive::MilliVolt MilliVoltMax{3100};

inline constexpr primitive::Position PositionMin{0};
inline constexpr primitive::Position PositionMax{1000};

inline constexpr primitive::ServoPosition ServoPositionMin{0};
inline constexpr primitive::ServoPosition ServoPositionMax{4100};

inline constexpr primitive::RPM RpmMin{0};
inline constexpr primitive::RPM RpmMax{9000};

inline constexpr primitive::Speed SpeedMin{0};
inline constexpr primitive::Speed SpeedMax{200};

inline constexpr primitive::Current CurrentMin{0};
inline constexpr primitive::Current CurrentMax{2500};

inline constexpr primitive::Temperature TemperatureMin{0};
inline constexpr primitive::Temperature TemperatureMax{150};

inline constexpr primitive::MilliSec MilliSecMin{0};
inline constexpr primitive::MilliSec MilliSecMax{1000};

}  // namespace type::limits
