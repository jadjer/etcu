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
#include "type/control.hpp"

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
    type::dto::ControlDTO const control_dto = control.to_dto();

    characteristic->setValue(control_dto);
  }

  auto onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) -> void override {
    auto const [cruise, servo_min, servo_max, accelerator_min, accelerator_max] = characteristic->getValue<type::dto::ControlDTO>();

    type::Control const control{
        .cruise =
            type::Cruise{
                .p = cruise.p,
                .i = cruise.i,
                .d = cruise.d,
                .rpm = {.min = cruise.rpm_min, .max = cruise.rpm_max},
                .speed = {.min = cruise.speed_min, .max = cruise.speed_max},
                .limiter_up = cruise.limiter_up,
                .limiter_down = cruise.limiter_down,
            },
        .servo = {.min = servo_min, .max = servo_max},
        .accelerator = {.min = accelerator_min, .max = accelerator_max},
    };

    m_container.store(control);
  }
};

}  // namespace bluetooth::callback
