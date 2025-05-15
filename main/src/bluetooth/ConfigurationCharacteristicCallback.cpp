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

#include "ConfigurationCharacteristicCallback.hpp"

#include "bluetooth/Identificator.hpp"

ConfigurationCharacteristicCallback::ConfigurationCharacteristicCallback(Configuration::Pointer const &configuration) : configuration(std::move(configuration)) {}

void ConfigurationCharacteristicCallback::onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) {
  NimBLECharacteristicCallbacks::onRead(pCharacteristic, connInfo);

  NimBLEUUID const uuid = pCharacteristic->getUUID();

  //  TODO Edit characteristics

  //  if (uuid == CHARACTERISTIC_PUMP_TIMEOUT_UUID) {
  //    uint64_t const pumpTimeout = m_configuration->getPumpTimeout();
  //    pCharacteristic->setValue<uint64_t>(pumpTimeout);
  //  }
  //
  //  //  if (uuid == CHARACTERISTIC_WHEEL_DIAMETER_UUID) {
  //  //    auto const pumpTimeout = m_configuration->getWheelLength();
  //  //    pCharacteristic->setValue(pumpTimeout);
  //  //  }
  //
  //  if (uuid == CHARACTERISTIC_MINIMAL_SPEED_UUID) {
  //    float const minimalSpeed = m_configuration->getMinimalSpeed();
  //    pCharacteristic->setValue<float>(minimalSpeed);
  //  }
  //
  //  if (uuid == CHARACTERISTIC_DISTANCE_FOR_ENABLE_UUID) {
  //    float const distanceForEnable = m_configuration->getDistanceForEnable();
  //    pCharacteristic->setValue<float>(distanceForEnable);
  //  }
  //
  //  if (uuid == CHARACTERISTIC_TOTAL_DISTANCE_UUID) {
  //    float const totalDistance = m_configuration->getTotalDistance();
  //    pCharacteristic->setValue<float>(totalDistance);
  //  }
  //
  //  if (uuid == CHARACTERISTIC_NEXT_DISTANCE_UUID) {
  //    float const nextDistance = m_configuration->getNextDistance();
  //    pCharacteristic->setValue<float>(nextDistance);
  //  }
  //
  //  if (uuid == CHARACTERISTIC_MANUAL_LUBRICATE_UUID) {
  //    bool const manualLubricate = m_configuration->isManualLubricate();
  //    pCharacteristic->setValue<bool>(manualLubricate);
  //  }
}

void ConfigurationCharacteristicCallback::onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) {
  NimBLECharacteristicCallbacks::onWrite(pCharacteristic, connInfo);

  NimBLEUUID const uuid = pCharacteristic->getUUID();

  //  TODO Edit characteristics

  //  if (uuid == CHARACTERISTIC_PUMP_TIMEOUT_UUID) {
  //    auto const pumpTimeout = pCharacteristic->getValue<uint64_t>();
  //    m_configuration->setPumpTimeout(pumpTimeout);
  //  }
  //
  //  //  if (uuid == CHARACTERISTIC_WHEEL_DIAMETER_UUID) {
  //  //    auto const pumpTimeout = m_configuration->getWheelLength();
  //  //    pCharacteristic->setValue(pumpTimeout);
  //  //  }
  //
  //  if (uuid == CHARACTERISTIC_MINIMAL_SPEED_UUID) {
  //    auto const minimalSpeed = pCharacteristic->getValue<float>();
  //    m_configuration->setMinimalSpeed(minimalSpeed);
  //  }
  //
  //  if (uuid == CHARACTERISTIC_DISTANCE_FOR_ENABLE_UUID) {
  //    auto const distanceForEnable = pCharacteristic->getValue<float>();
  //    m_configuration->setDistanceForEnable(distanceForEnable);
  //  }
  //
  //  if (uuid == CHARACTERISTIC_MANUAL_LUBRICATE_UUID) {
  //    auto const manualLubricate = pCharacteristic->getValue<bool>();
  //    m_configuration->setManualLubricate(manualLubricate);
  //  }
}

void ConfigurationCharacteristicCallback::onStatus(NimBLECharacteristic *pCharacteristic, int code) { NimBLECharacteristicCallbacks::onStatus(pCharacteristic, code); }

void ConfigurationCharacteristicCallback::onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) {
  NimBLECharacteristicCallbacks::onSubscribe(pCharacteristic, connInfo, subValue);
}
