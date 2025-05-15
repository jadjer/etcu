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

#include "bluetooth/Bluetooth.hpp"

#include <NimBLEDevice.h>

#include <utility>

#include "ConfigurationCharacteristicCallback.hpp"
#include "ServerCallback.hpp"
#include "bluetooth/Identificator.hpp"
#include "ota/MessageHandler.hpp"
#include "ota/UpdateCharacteristicCallback.hpp"

namespace {

auto const MTU = BLE_ATT_MTU_MAX;

} // namespace

Bluetooth::Bluetooth(Configuration::Pointer const &configuration)
    : deviceName("ETCU"), serverCallback(std::make_unique<ServerCallback>()), otaCharacteristicCallback(nullptr),
      configurationCharacteristicCallback(std::make_unique<ConfigurationCharacteristicCallback>(configuration)) {

//  TODO Get name from config

  NimBLEDevice::init(deviceName);
  NimBLEDevice::setMTU(MTU);
  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
  NimBLEDevice::setSecurityPasskey(586226);

  auto *const server = NimBLEDevice::createServer();
  server->setCallbacks(serverCallback.get());
  server->advertiseOnDisconnect(true);

  {
    auto *const service = server->createService(SERVICE_CONFIGURATION_UUID);

    auto *const pumpTimeoutCharacteristic = service->createCharacteristic(CHARACTERISTIC_PUMP_TIMEOUT_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    auto *const minimalSpeedCharacteristic = service->createCharacteristic(CHARACTERISTIC_MINIMAL_SPEED_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    auto *const distanceForEnableCharacteristic = service->createCharacteristic(CHARACTERISTIC_DISTANCE_FOR_ENABLE_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    auto *const totalDistanceCharacteristic = service->createCharacteristic(CHARACTERISTIC_TOTAL_DISTANCE_UUID, NIMBLE_PROPERTY::READ);
    auto *const nextDistanceCharacteristic = service->createCharacteristic(CHARACTERISTIC_NEXT_DISTANCE_UUID, NIMBLE_PROPERTY::READ);

    pumpTimeoutCharacteristic->setCallbacks(configurationCharacteristicCallback.get());
    minimalSpeedCharacteristic->setCallbacks(configurationCharacteristicCallback.get());
    distanceForEnableCharacteristic->setCallbacks(configurationCharacteristicCallback.get());
    totalDistanceCharacteristic->setCallbacks(configurationCharacteristicCallback.get());
    nextDistanceCharacteristic->setCallbacks(configurationCharacteristicCallback.get());

    service->start();
  }

  {
    auto *const service = server->createService(SERVICE_CONTROL_UUID);

    auto *const manualLubricateCharacteristic = service->createCharacteristic(CHARACTERISTIC_MANUAL_LUBRICATE_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    manualLubricateCharacteristic->setCallbacks(configurationCharacteristicCallback.get());

    service->start();
  }

  {
    auto *const service = server->createService(SERVICE_OTA_UUID);

    auto *const dataCharacteristic = service->createCharacteristic(CHARACTERISTIC_DATA_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    auto *const commandCharacteristic = service->createCharacteristic(CHARACTERISTIC_COMMAND_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);

    otaCharacteristicCallback = std::make_unique<UpdateCharacteristicCallback>(std::make_unique<MessageHandler>(dataCharacteristic, commandCharacteristic));

    dataCharacteristic->setCallbacks(otaCharacteristicCallback.get());
    commandCharacteristic->setCallbacks(otaCharacteristicCallback.get());

    service->start();
  }

  server->start();
}

Bluetooth::~Bluetooth() { NimBLEDevice::deinit(); }

void Bluetooth::advertise() {
  auto *const advertising = NimBLEDevice::getAdvertising();
  advertising->setName(deviceName);
  advertising->setManufacturerData("jadjer");
  advertising->enableScanResponse(true);
  advertising->addServiceUUID(ADVERTISING_UUID);
  advertising->start();
}
