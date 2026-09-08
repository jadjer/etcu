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
// Created by jadjer on 8.09.26.
//

#pragma once

#include <NimBLECharacteristic.h>

#include "common/atomic_container.hpp"
#include "config/constants.hpp"
#include "type/telemetry.hpp"

namespace bluetooth::callback {

class CalibrationCallback : public NimBLECharacteristicCallbacks {
public:
  constexpr explicit CalibrationCallback() noexcept {}

  CalibrationCallback(CalibrationCallback const&) noexcept = delete;
  auto operator=(CalibrationCallback const&) noexcept -> CalibrationCallback& = delete;

  CalibrationCallback(CalibrationCallback&&) noexcept = delete;
  auto operator=(CalibrationCallback&&) noexcept -> CalibrationCallback& = delete;

  ~CalibrationCallback() noexcept override = default;

  auto onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) -> void override {
    ESP_LOGI("CAL", "WRITE");
  }
};

}  // namespace bluetooth::callback
