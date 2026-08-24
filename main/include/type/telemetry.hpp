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

struct BluetoothControl {
  bool sync_enabled{false};
  Position accelerator_offset{0};

  constexpr BluetoothControl() noexcept = default;

  constexpr explicit BluetoothControl(dto::BluetoothControl const& dto) noexcept : sync_enabled{dto.sync_enabled}, accelerator_offset(dto.accelerator_offset) {}

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::BluetoothControl {
    return dto::BluetoothControl{
        .sync_enabled = sync_enabled,
        .accelerator_offset = accelerator_offset.value,
    };
  }
};

template <std::uint8_t PackageSize>
struct OTAChunk {
  static constexpr std::uint8_t package_size{PackageSize};

  std::array<std::uint8_t, package_size> chunk{};
  std::uint16_t chunk_size{0};
  std::uint16_t chunk_number{0};
  std::uint16_t chunk_total{0};
  std::uint32_t firmware_size{0};

  constexpr OTAChunk() noexcept = default;

  constexpr explicit OTAChunk(dto::OTAChunk<PackageSize> const& dto) noexcept
      : chunk{dto.chunk}, chunk_size(dto.chunk_size), chunk_number(dto.chunk_number), chunk_total(dto.chunk_total), firmware_size(dto.firmware_size) {}
};

struct ServoTelemetry {
  bool is_connected{false};
  bool is_moved{false};

  primitive::Load load{0};
  Speed speed{0};
  Current current{0};
  Volt voltage{0};
  ServoPosition position{0};
  Temperature temperature{0};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::ServoTelemetry {
    return dto::ServoTelemetry{
        .is_connected = is_connected,
        .is_moved = is_moved,

        .load = load,
        .speed = speed.value,
        .current = current.value,
        .voltage = voltage.value,
        .position = position.value,
        .temperature = temperature.value,
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

        .rpm = rpm.value,
        .speed = speed.value,
        .tps = tps.value,
    };
  }
};

struct SystemTelemetry {
  bool is_guard_active{false};
  bool is_brake_enabled{false};
  bool is_clutch_enabled{false};

  ServoTelemetry servo_telemetry{};
  ECUTelemetry ecu_telemetry{};

  Position accelerator_position{0};
  Position accelerator_offset{0};
  Position throttle_position{0};
  Speed target_speed{0};

  SystemState system_state{SystemState::Off};
  SystemError system_errors{SystemError::None};

  [[nodiscard]] constexpr auto to_dto() const noexcept -> dto::SystemTelemetry {
    return dto::SystemTelemetry{
        .is_guard_active = is_guard_active,
        .is_brake_enabled = is_brake_enabled,
        .is_clutch_enabled = is_clutch_enabled,

        .servo_telemetry = servo_telemetry.to_dto(),
        .ecu_telemetry = ecu_telemetry.to_dto(),

        .accelerator_position = accelerator_position.value,
        .accelerator_offset = accelerator_offset.value,
        .throttle_position = throttle_position.value,
        .target_speed = target_speed.value,

        .system_state = system_state,
        .system_errors = system_errors,
    };
  }
};

}  // namespace type
