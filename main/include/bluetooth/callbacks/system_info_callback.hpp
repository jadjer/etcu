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
