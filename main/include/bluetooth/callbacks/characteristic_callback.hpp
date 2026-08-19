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
#include "common/atomic_channel.hpp"

namespace bluetooth::callback {

template <typename DataType>
class CharacteristicCallback : public NimBLECharacteristicCallbacks {
  common::AtomicChannel<DataType>& m_channel;

 public:
  constexpr explicit CharacteristicCallback(common::AtomicChannel<DataType>& channel) : m_channel(channel) {}

  constexpr auto onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& /*conn_info*/) -> void override {
    DataType const chunk = characteristic->getValue<DataType>();

    std::ignore = m_channel.send(chunk);
  }
};

}  // namespace bluetooth::callback
