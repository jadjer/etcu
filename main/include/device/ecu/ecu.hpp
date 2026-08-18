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

#include "concepts.hpp"

namespace device {

template <typename Driver>
  requires concepts::UART<Driver>

class ECU {
  Driver& m_driver;

 public:
  explicit ECU(Driver& driver) noexcept : m_driver{driver} {}

  auto init() noexcept -> type::SystemError {
    std::ignore = m_driver.init();
    return type::SystemError::None;
  }

  auto update() noexcept -> type::SystemError { return type::SystemError::None; }  // NOLINT

  [[nodiscard]] auto get_telemetry(type::ECUTelemetry& telemetry) const noexcept -> type::SystemError {  // NOLINT
    telemetry.is_connected = false;
    telemetry.rpm = 0;
    telemetry.speed = 0;
    telemetry.tps = 0;
    telemetry.started = false;
    telemetry.clutch_enabled = false;

    return type::SystemError::None;
  }
};

}  // namespace device
