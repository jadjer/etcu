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

#include <array>

#include "type/error.hpp"
#include "type/state.hpp"
#include "type/telemetry_dto.hpp"
#include "type/type.hpp"

namespace type {

struct PositionRange {
  Position min{Position::value_min};
  Position max{Position::value_max};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::PositionRangeDTO {
    return dto::PositionRangeDTO{
        .min = min.get(),
        .max = max.get(),
    };
  }

  [[nodiscard]] auto operator<=>(PositionRange const&) const = default;
};

struct Control {
  static constexpr std::string_view name{"control"};
  static constexpr std::uint32_t current_version{1};

  std::uint32_t version{0};

  PositionRange servo{};
  PositionRange accelerator{};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::ControlDTO {
    return dto::ControlDTO{
        .servo = servo.to_dto(),
        .accelerator = accelerator.to_dto(),
    };
  }

  [[nodiscard]] auto operator<=>(Control const&) const = default;
};

template <std::size_t PayloadSize>
struct OTAChunk {
  std::uint32_t firmware_size{0};
  std::uint16_t chunk_total{0};
  std::uint16_t chunk_index{0};

  std::array<std::uint8_t, PayloadSize> payload{};
};

struct ServoTelemetry {
  bool is_connected{false};
  bool is_enabled{false};
  bool is_moved{false};

  Volt voltage{0};
  Current current{0};
  ServoPosition position{0};
  Temperature temperature{0};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::ServoTelemetryDTO {
    return dto::ServoTelemetryDTO{
        .is_connected = is_connected,
        .is_enabled = is_enabled,
        .is_moved = is_moved,

        .voltage = voltage.get(),
        .current = current.get(),
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
  Volt battery{0};
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
        .battery = battery.get(),
        .speed = speed.get(),
        .map = map.get(),
        .tps = tps.get(),
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
        .hall_b = hall_a.get(),
        .position = position.get(),
    };
  }
};

struct SystemTelemetry {
  bool is_guard_active{false};
  bool is_brake_enabled{false};

  ECUTelemetry ecu_telemetry{};
  ServoTelemetry servo_telemetry{};
  AcceleratorTelemetry accelerator_telemetry{};

  Speed target_speed{0};
  Position throttle_position{0};

  SystemState system_state{SystemState::Off};
  SystemError system_errors{SystemError::None};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::SystemTelemetryDTO {
    return dto::SystemTelemetryDTO{
        .is_guard_active = is_guard_active,
        .is_brake_enabled = is_brake_enabled,

        .ecu_telemetry = ecu_telemetry.to_dto(),
        .servo_telemetry = servo_telemetry.to_dto(),
        .accelerator_telemetry = accelerator_telemetry.to_dto(),

        .target_speed = target_speed.get(),
        .throttle_position = throttle_position.get(),

        .system_state = system_state,
        .system_errors = system_errors,
    };
  }
};

struct DriveTelemetry {
  Position throttle_position{};
  ServoTelemetry servo_telemetry{};
  AcceleratorTelemetry accelerator_telemetry{};
};

}  // namespace type
