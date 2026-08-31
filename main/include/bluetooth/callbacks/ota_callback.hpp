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
#include "config/constants.hpp"
#include "type/telemetry.hpp"

namespace bluetooth::callback {

class OTACallback : public NimBLECharacteristicCallbacks {
  common::AtomicContainer<type::OTAChunk<constants::bluetooth::OTAPayloadSize>>& m_container;

 public:
  constexpr explicit OTACallback(common::AtomicContainer<type::OTAChunk<constants::bluetooth::OTAPayloadSize>>& container) noexcept : m_container(container) {}

  OTACallback(OTACallback const&) noexcept = delete;
  auto operator=(OTACallback const&) noexcept -> OTACallback& = delete;

  OTACallback(OTACallback&&) noexcept = delete;
  auto operator=(OTACallback&&) noexcept -> OTACallback& = delete;

  ~OTACallback() noexcept override = default;

  auto onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) -> void override {
    auto const [firmware_size, total, index, data] = characteristic->getValue<type::dto::OTAChunk<constants::bluetooth::OTAPayloadSize>>();

    auto const chunk = type::OTAChunk{
        .firmware_size = firmware_size,
        .chunk_total = total,
        .chunk_index = index,
        .payload = data,
    };

    m_container.store(chunk);
  }
};

}  // namespace bluetooth::callback
