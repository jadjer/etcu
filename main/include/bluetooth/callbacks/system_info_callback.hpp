//
// Created by jadjer on 23.08.26.
//

#pragma once

#include <NimBLECharacteristic.h>
#include "type/telemetry.hpp"

#include "common/to_fixed_string.hpp"

namespace bluetooth::callback {

struct SystemInfoCallback : NimBLECharacteristicCallbacks {
  static constexpr type::dto::SystemInfo system_info{
      .build_date{common::to_fixed_string<16>(BUILD_DATE)},
      .board_version{common::to_fixed_string<16>(BOARD_VERSION)},
      .firmware_version{common::to_fixed_string<16>(FW_VERSION)},
  };

  void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    characteristic->setValue(reinterpret_cast<uint8_t const*>(&system_info), sizeof(system_info));
  }
};

}  // namespace bluetooth::callback
