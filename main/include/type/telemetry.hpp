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

#include "type/dto.hpp"
#include "type/error.hpp"
#include "type/state.hpp"
#include "type/type.hpp"

namespace type {

struct ServoTelemetry {
  bool is_connected{false};
  bool is_enabled{false};
  bool is_moved{false};

  Current current{0};
  Voltage voltage{0};
  ServoPosition position{0};
  Temperature temperature{0};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::ServoTelemetryDTO {
    return dto::ServoTelemetryDTO{
        .is_connected = is_connected,
        .is_enabled = is_enabled,
        .is_moved = is_moved,

        .current = current.get(),
        .voltage = voltage.get(),
        .position = position.get(),
        .temperature = temperature.get(),
    };
  }
};

struct ECUTelemetry {
  bool is_connected{false};
  bool is_started{false};
  bool is_neutral{false};

  RPM rpm{0};
  Voltage battery{0};
  Speed speed{0};
  Pressure map{0};
  Position tps{0};
  Temperature air{0};
  Temperature coolant{0};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::ECUTelemetryDTO {
    return dto::ECUTelemetryDTO{
        .is_connected = is_connected,
        .is_started = is_started,
        .is_neutral = is_neutral,

        .rpm = rpm.get(),
        .speed = speed.get(),
        .map = map.get(),
        .tps = tps.get(),
        .battery = battery.get(),
        .air = air.get(),
        .coolant = coolant.get(),
    };
  }
};

struct AcceleratorTelemetry {
  AccPosition hall_a{0};
  AccPosition hall_b{0};
  Position position{0};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::AcceleratorTelemetryDTO {
    return dto::AcceleratorTelemetryDTO{
        .hall_a = hall_a.get(),
        .hall_b = hall_b.get(),
        .position = position.get(),
    };
  }
};

struct CruiseTelemetry {
  bool is_enabled{false};
  bool is_activated{false};

  float error{0.0f};
  float correction{0};

  Speed target_speed{0};
  Speed current_speed{0};

  Position base_position{0};
  Position current_position{0};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::CruiseTelemetryDTO {
    return dto::CruiseTelemetryDTO{
        .is_enabled = is_enabled,
        .is_activated = is_activated,

        .error = error,
        .correction = correction,

        .target_speed = target_speed.get(),
        .current_speed = current_speed.get(),

        .base_position = base_position.get(),
        .current_position = current_position.get(),
    };
  }
};

struct SystemTelemetry {
  bool is_guard_active{false};
  bool is_brake_enabled{false};

  Position throttle_position{0};

  ECUTelemetry ecu_telemetry{};
  ServoTelemetry servo_telemetry{};
  CruiseTelemetry cruise_telemetry{};
  AcceleratorTelemetry accelerator_telemetry{};

  SystemState system_state{SystemState::Off};
  SystemError system_errors{SystemError::None};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::SystemTelemetryDTO {
    return dto::SystemTelemetryDTO{
        .is_guard_active = is_guard_active,
        .is_brake_enabled = is_brake_enabled,

        .throttle_position = throttle_position.get(),

        .ecu_telemetry = ecu_telemetry.to_dto(),
        .servo_telemetry = servo_telemetry.to_dto(),
        .cruise_telemetry = cruise_telemetry.to_dto(),
        .accelerator_telemetry = accelerator_telemetry.to_dto(),

        .system_state = system_state,
        .system_errors = system_errors,
    };
  }
};

}  // namespace type
