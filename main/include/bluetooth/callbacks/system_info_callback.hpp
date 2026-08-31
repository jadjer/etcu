//
// Created by jadjer on 23.08.26.
//

#pragma once

#include <NimBLECharacteristic.h>
#include "type/telemetry.hpp"

#include "common/to_fixed_string.hpp"

namespace bluetooth::callback {

class SystemInfoCallback : public NimBLECharacteristicCallbacks {
  static constexpr type::dto::SystemInfo system_info{
      .build_date{common::to_fixed_string<16>(BUILD_DATE)},
      .board_version{common::to_fixed_string<16>(BOARD_VERSION)},
      .firmware_version{common::to_fixed_string<16>(FW_VERSION)},
  };

 public:
  constexpr SystemInfoCallback() noexcept = default;

  SystemInfoCallback(SystemInfoCallback const&) noexcept = delete;
  auto operator=(SystemInfoCallback const&) noexcept -> SystemInfoCallback& = delete;

  SystemInfoCallback(SystemInfoCallback&&) noexcept = delete;
  auto operator=(SystemInfoCallback&&) noexcept -> SystemInfoCallback& = delete;

  ~SystemInfoCallback() noexcept override = default;

  void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override { characteristic->setValue(system_info); }
};

}  // namespace bluetooth::callback
