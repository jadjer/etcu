//
// Created by jadjer on 23.08.26.
//

#pragma once

#include <NimBLECharacteristic.h>
#include "type/telemetry.hpp"

#include "common/to_fixed_string.hpp"

namespace bluetooth::callback {

struct SystemInfoCallback : NimBLECharacteristicCallbacks {
  static constexpr type::dto::SystemInfo SYSTEM_INFO{.board_version{common::to_fixed_string<16>(BOARD_VERSION)},
                                                         .build_date{common::to_fixed_string<16>(BUILD_DATE)},
                                                         .firmware_version{common::to_fixed_string<16>(FW_VERSION)}};

  void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    pCharacteristic->setValue(reinterpret_cast<uint8_t const*>(&SYSTEM_INFO), sizeof(type::dto::SystemInfo));
  }
};

}  // namespace bluetooth::callback
