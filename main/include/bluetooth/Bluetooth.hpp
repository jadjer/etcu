// Copyright 2025 Pavel Suprunov
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

#pragma once

#include "configuration/interface/Configuration.hpp"

#include <NimBLECharacteristic.h>
#include <NimBLEServer.h>
#include <memory>

class Bluetooth {
public:
  using DeviceName = std::string;
  using ServerCallbackPtr = std::unique_ptr<NimBLEServerCallbacks>;
  using CharacteristicCallbackPtr = std::unique_ptr<NimBLECharacteristicCallbacks>;

public:
  Bluetooth(ConfigurationSharedPtr const &configuration, DeviceName deviceName);
  ~Bluetooth();

public:
  auto advertise() -> void;

private:
  DeviceName const deviceName{};

private:
  ServerCallbackPtr serverCallback{nullptr};
  CharacteristicCallbackPtr otaCharacteristicCallback{nullptr};
  CharacteristicCallbackPtr configurationCharacteristicCallback{nullptr};
};
