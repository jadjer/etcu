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
#include "type/calibration.hpp"

namespace type {
struct Calibration;
}
namespace bluetooth::callback {

class CalibrationCallback : public NimBLECharacteristicCallbacks {
  common::AtomicContainer<type::Calibration>& m_calibration;

 public:
  constexpr explicit CalibrationCallback(common::AtomicContainer<type::Calibration>& calibration) noexcept : m_calibration(calibration) {}

  CalibrationCallback(CalibrationCallback const&) noexcept = delete;
  auto operator=(CalibrationCallback const&) noexcept -> CalibrationCallback& = delete;

  CalibrationCallback(CalibrationCallback&&) noexcept = delete;
  auto operator=(CalibrationCallback&&) noexcept -> CalibrationCallback& = delete;

  ~CalibrationCallback() noexcept override = default;

  auto onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo&) -> void override {
    type::Calibration const calibration = m_calibration.load();
    type::dto::CalibrationDTO const calibration_dto = calibration.to_dto();

    characteristic->setValue(calibration_dto);
  }

  auto onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) -> void override {
    const auto [hall_a_min, hall_a_max, hall_b_min, hall_b_max, servo_min, servo_max] = characteristic->getValue<type::dto::CalibrationDTO>();

    type::Calibration const calibration{
        .servo =
            type::ServoCalibrationData{
                .position_minimal = servo_min,
                .position_maximal = servo_max,
            },
        .accelerator =
            type::AcceleratorCalibrationData{
                .hall_a_minimal = hall_a_min,
                .hall_a_maximal = hall_a_max,
                .hall_b_minimal = hall_b_min,
                .hall_b_maximal = hall_b_max,
            },
    };

    m_calibration.store(calibration);
  }
};

}  // namespace bluetooth::callback
