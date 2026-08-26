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

namespace type::dto {

struct BluetoothControl {
  bool sync_enabled{false};

  primitive::Position accelerator_offset{0};

} __attribute__((packed));

struct ECUTelemetry {
  bool is_connected{false};
  bool is_started{false};
  bool is_clutch_enabled{false};

  primitive::RPM rpm{0};
  primitive::Speed speed{0};
  primitive::Position tps{0};

} __attribute__((packed));

template <std::uint8_t PackageSize>
struct OTAChunk {
  static constexpr std::uint8_t package_size{PackageSize};

  std::uint32_t firmware_size{0};
  std::uint16_t chunk_total{0};
  std::uint16_t chunk_number{0};
  std::uint16_t chunk_size{0};

  std::array<std::uint8_t, package_size> chunk{};

} __attribute__((packed));

struct ServoTelemetry {
  bool is_connected{false};
  bool is_moved{false};

  primitive::Speed speed{0};
  primitive::Current current{0};
  primitive::Volt voltage{0};
  primitive::ServoPosition position{0};
  primitive::Temperature temperature{0};

} __attribute__((packed));

struct SystemInfo {
  primitive::FixedString board_version{};
  primitive::FixedString build_date{};
  primitive::FixedString firmware_version{};

} __attribute__((packed));

struct SystemTelemetry {
  bool is_guard_active{false};
  bool is_brake_enabled{false};

  ECUTelemetry ecu_telemetry{};
  ServoTelemetry servo_telemetry{};

  primitive::Position accelerator_position{0};
  primitive::Position accelerator_offset{0};
  primitive::Position throttle_position{0};
  primitive::Speed target_speed{0};

  SystemState system_state{SystemState::Off};
  SystemError system_errors{SystemError::None};

} __attribute__((packed));

}
