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

namespace type {

enum class SystemState : primitive::Byte {
  Off = 0,
  Normal,
  Calibration,
  Update,
};

enum class OTAStatus : primitive::Byte {
  ReadyForNext = 0x00,
  Busy = 0x01,
  ErrorOccurred = 0x02,
  Completed = 0x03,
};

enum class ServoMode : primitive::Byte {
  PositionMode = 0x00,
  WheelMode = 0x01,
  PwmMode = 0x02,
  StepMode = 0x03,
};

}

