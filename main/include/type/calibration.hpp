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

#include <string_view>
#include "type/type.hpp"

namespace type {

struct AcceleratorCalibrationData {
  static constexpr std::string_view struct_name{"acc_calib"};
  static constexpr std::uint32_t current_version{0x10000001};

  std::uint32_t struct_version{current_version};

  MilliVolt hall_a_minimal{0};
  MilliVolt hall_a_maximal{0};
  MilliVolt hall_b_minimal{0};
  MilliVolt hall_b_maximal{0};
};

struct ServoCalibrationData {
  static constexpr std::string_view struct_name{"servo_calib"};
  static constexpr std::uint32_t current_version{0x10000001};

  std::uint32_t struct_version{current_version};

  ServoPosition position_minimal{0};
  ServoPosition position_maximal{0};
};

}  // namespace type
