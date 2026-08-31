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

struct Control {
  Position servo_min{Position::value_min};
  Position servo_max{Position::value_max};
  Position accelerator_min{Position::value_min};
  Position accelerator_max{Position::value_max};
  Position accelerator_dead_min{Position::value_min};
  Position accelerator_dead_max{Position::value_max};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::Control {
    return dto::Control{
        .servo_min = servo_min.get(),
        .servo_max = servo_max.get(),
        .accelerator_min = accelerator_min.get(),
        .accelerator_max = accelerator_max.get(),
        .accelerator_dead_min = accelerator_dead_min.get(),
        .accelerator_dead_max = accelerator_dead_max.get(),
    };
  }
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

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::ServoTelemetry {
    return dto::ServoTelemetry{
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
  bool is_clutch_enabled{false};

  RPM rpm{0};
  Speed speed{0};
  Position tps{0};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::ECUTelemetry {
    return dto::ECUTelemetry{
        .is_connected = is_connected,
        .is_started = is_started,
        .is_clutch_enabled = is_clutch_enabled,

        .rpm = rpm.get(),
        .speed = speed.get(),
        .tps = tps.get(),
    };
  }
};

struct SystemTelemetry {
  bool is_guard_active{false};
  bool is_brake_enabled{false};

  ServoTelemetry servo_telemetry{};
  ECUTelemetry ecu_telemetry{};

  Position accelerator_position{0};
  Position throttle_position{0};
  Speed target_speed{0};

  SystemState system_state{SystemState::Off};
  SystemError system_errors{SystemError::None};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::SystemTelemetry {
    return dto::SystemTelemetry{
        .is_guard_active = is_guard_active,
        .is_brake_enabled = is_brake_enabled,

        .ecu_telemetry = ecu_telemetry.to_dto(),
        .servo_telemetry = servo_telemetry.to_dto(),

        .target_speed = target_speed.get(),
        .throttle_position = throttle_position.get(),
        .accelerator_position = accelerator_position.get(),

        .system_state = system_state,
        .system_errors = system_errors,
    };
  }
};

struct DriveTelemetry {
  Position throttle_position{};
  Position accelerator_position{};
  ServoTelemetry servo_telemetry{};
};

}  // namespace type
