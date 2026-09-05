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
// Created by jadjer on 28.07.26.
//

#pragma once

#include <NimBLECharacteristic.h>
#include "common/atomic_container.hpp"
#include "type/telemetry.hpp"

namespace bluetooth::callback {

class ControlCallback : public NimBLECharacteristicCallbacks {
  common::AtomicContainer<type::Control>& m_container;

 public:
  constexpr explicit ControlCallback(common::AtomicContainer<type::Control>& control) : m_container(control) {}

  ControlCallback(ControlCallback const&) noexcept = delete;
  auto operator=(ControlCallback const&) noexcept -> ControlCallback& = delete;

  ControlCallback(ControlCallback&&) noexcept = delete;
  auto operator=(ControlCallback&&) noexcept -> ControlCallback& = delete;

  ~ControlCallback() noexcept override = default;

  auto onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo&) -> void override {
    type::Control const control = m_container.load();
    type::dto::Control const control_dto = control.to_dto();

    characteristic->setValue(control_dto);
  }

  auto onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) -> void override {
    auto const [auto_set, servo, accelerator] = characteristic->getValue<type::dto::Control>();

    type::Control const control{
        .auto_set =
            type::Control::AutoSet{
                .enabled = auto_set.enabled,
                .delay = auto_set.delay,
                .threshold = auto_set.threshold,
                .tolerance = auto_set.tolerance,
            },
        .servo =
            type::Control::Range{
                .min = servo.min,
                .max = servo.max,
            },
        .accelerator =
            type::Control::Range{
                .min = accelerator.min,
                .max = accelerator.max,
            },
    };

    m_container.store(control);
  }
};

}  // namespace bluetooth::callback
