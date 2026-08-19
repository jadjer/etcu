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

template <class Driver>
  requires concepts::UART<Driver>

class ECU {
  Driver& m_driver;

 public:
  constexpr explicit ECU(Driver& driver) noexcept : m_driver{driver} {}

  constexpr auto init() noexcept -> type::SystemError {
    std::ignore = m_driver.init();
    return type::SystemError::None;
  }

  constexpr auto update() noexcept -> type::SystemError { return type::SystemError::None; }  // NOLINT

  [[nodiscard]] constexpr auto get_telemetry(type::ECUTelemetry& telemetry) const noexcept -> type::SystemError {  // NOLINT
    telemetry.is_connected = false;
    telemetry.rpm = type::RPM{0};
    telemetry.speed = type::Speed{0};
    telemetry.tps = type::Position{0};
    telemetry.started = false;
    telemetry.clutch_enabled = false;

    return type::SystemError::None;
  }
};

}  // namespace device
