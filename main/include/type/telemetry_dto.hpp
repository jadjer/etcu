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
#include "type/primitive.hpp"
#include "type/state.hpp"

namespace type::dto {

struct CruiseAutoSetDTO {
  bool enabled{false};            // 1
  std::uint8_t delay{0};          // 1
  primitive::Speed threshold{0};  // 1
  primitive::Speed tolerance{0};  // 1

} __attribute__((packed));

struct PositionRangeDTO {
  primitive::Position min{0};  // 2
  primitive::Position max{0};  // 2

} __attribute__((packed));

struct ControlDTO {
  CruiseAutoSetDTO cruise;       // 4
  PositionRangeDTO servo;        // 4
  PositionRangeDTO accelerator;  // 4

} __attribute__((packed));

template <std::size_t PayloadSize>
struct OTAChunkDTO {
  std::uint32_t firmware_size{0};  // 4
  std::uint16_t chunk_total{0};    // 2
  std::uint16_t chunk_index{0};    // 2

  std::array<std::uint8_t, PayloadSize> chunk{};

} __attribute__((packed));

struct CalibrationDTO {
  primitive::AccPosition hall_a_min{0};   // 2
  primitive::AccPosition hall_a_max{0};   // 2
  primitive::AccPosition hall_b_min{0};   // 2
  primitive::AccPosition hall_b_max{0};   // 2
  primitive::ServoPosition servo_min{0};  // 2
  primitive::ServoPosition servo_max{0};  // 2

} __attribute__((packed));

struct SystemInfoDTO {
  primitive::FixedString build_date{};        // 16
  primitive::FixedString board_version{};     // 16
  primitive::FixedString firmware_version{};  // 16

} __attribute__((packed));

struct ECUTelemetryDTO {
  bool is_connected{false};  // 1
  bool is_started{false};    // 1
  bool is_neutral{false};    // 1

  primitive::RPM rpm{0};              // 2
  primitive::Volt battery{0};         // 1
  primitive::Speed speed{0};          // 1
  primitive::Pressure map{0};         // 1
  primitive::Position tps{0};         // 2
  primitive::Temperature air{0};      // 1
  primitive::Temperature coolant{0};  // 1

} __attribute__((packed));

struct ServoTelemetryDTO {
  bool is_connected{false};  // 1
  bool is_enabled{false};    // 1
  bool is_moved{false};      // 1

  primitive::Volt voltage{0};             // 1
  primitive::Current current{0};          // 2
  primitive::ServoPosition position{0};   // 2
  primitive::Temperature temperature{0};  // 1

} __attribute__((packed));

struct AcceleratorTelemetryDTO {
  primitive::AccPosition hall_a{0};  // 2
  primitive::AccPosition hall_b{0};  // 2
  primitive::Position position{0};   // 2

} __attribute__((packed));

struct SystemTelemetryDTO {
  bool is_guard_active{false};   // 1
  bool is_brake_enabled{false};  // 1

  ECUTelemetryDTO ecu_telemetry{};                  // 12
  ServoTelemetryDTO servo_telemetry{};              // 9
  AcceleratorTelemetryDTO accelerator_telemetry{};  // 6

  primitive::Speed target_speed{0};          // 1
  primitive::Position throttle_position{0};  // 2

  SystemState system_state{SystemState::Off};    // 1
  SystemError system_errors{SystemError::None};  // 2

} __attribute__((packed));

}  // namespace type::dto
