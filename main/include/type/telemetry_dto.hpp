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

struct Control {
  struct AutoSet {                  // 1
    bool enabled{false};            // 1
    primitive::Time delay{0};       // 2
    primitive::Speed threshold{0};  // 1
    primitive::Speed tolerance{0};  // 1

  } __attribute__((packed));

  struct Range {                 // 1
    primitive::Position min{0};  // 2
    primitive::Position max{0};  // 2

  } __attribute__((packed));

  AutoSet auto_set;   // 6
  Range servo;        // 5
  Range accelerator;  // 5

} __attribute__((packed));

template <std::size_t PayloadSize>
struct OTAChunk {
  std::uint32_t firmware_size{0};  // 4
  std::uint16_t chunk_total{0};    // 2
  std::uint16_t chunk_index{0};    // 2

  std::array<std::uint8_t, PayloadSize> chunk{};

} __attribute__((packed));

struct SystemInfo {
  primitive::FixedString build_date{};        // 16
  primitive::FixedString board_version{};     // 16
  primitive::FixedString firmware_version{};  // 16

} __attribute__((packed));

struct ECUTelemetry {
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

struct ServoTelemetry {
  bool is_connected{false};  // 1
  bool is_enabled{false};    // 1
  bool is_moved{false};      // 1

  primitive::Volt voltage{0};             // 1
  primitive::Current current{0};          // 2
  primitive::ServoPosition position{0};   // 2
  primitive::Temperature temperature{0};  // 1

} __attribute__((packed));

struct SystemTelemetry {
  bool is_guard_active{false};   // 1
  bool is_brake_enabled{false};  // 1

  ECUTelemetry ecu_telemetry{};      // 12
  ServoTelemetry servo_telemetry{};  // 9

  primitive::Speed target_speed{0};             // 1
  primitive::Position throttle_position{0};     // 2
  primitive::Position accelerator_position{0};  // 2

  SystemState system_state{SystemState::Off};    // 1
  SystemError system_errors{SystemError::None};  // 4

} __attribute__((packed));

}  // namespace type::dto
