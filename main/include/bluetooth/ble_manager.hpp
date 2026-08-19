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
// Created by jadjer on 27.07.26.
//

#pragma once

#include <NimBLEDevice.h>
#include "bluetooth/callbacks/characteristic_callback.hpp"
#include "bluetooth/callbacks/server_callback.hpp"
#include "common/atomic_channel.hpp"
#include "constants.hpp"
#include "type.hpp"

namespace bluetooth {

class BLEManager {
  NimBLEServer* m_server{nullptr};
  NimBLECharacteristic* m_ota_characteristic{nullptr};
  NimBLECharacteristic* m_control_characteristic{nullptr};
  NimBLECharacteristic* m_telemetry_characteristic{nullptr};

  callback::ServerCallback m_server_callback;
  callback::CharacteristicCallback<type::OTAChunk<constants::bluetooth::MAX_BLE_PAYLOAD_SIZE>> m_ota_callback;
  callback::CharacteristicCallback<type::BluetoothControl> m_control_callback;

 public:
  explicit BLEManager(common::AtomicChannel<type::OTAChunk<constants::bluetooth::MAX_BLE_PAYLOAD_SIZE>>& ota_chunk, common::AtomicChannel<type::BluetoothControl>& control)
      : m_ota_callback(ota_chunk), m_control_callback(control) {}

  auto init() -> type::SystemError {
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    if (!NimBLEDevice::init(constants::bluetooth::DEVICE_NAME)) {
      return type::SystemError::BluetoothInitFault;
    }

    if (!NimBLEDevice::setPower(ESP_PWR_LVL_P9)) {
      return type::SystemError::BluetoothInitFault | type::SystemError::BluetoothSetPowerFault;
    }

    if (!NimBLEDevice::setMTU(512)) {
      return type::SystemError::BluetoothInitFault | type::SystemError::BluetoothSetMTUFault;
    }

    m_server = NimBLEDevice::createServer();
    m_server->setCallbacks(&m_server_callback);

    NimBLEService* service = m_server->createService(constants::bluetooth::SERVICE_UUID);
    {
      m_ota_characteristic = service->createCharacteristic(constants::bluetooth::OTA_CHARACTERISTIC_UUID, WRITE | NOTIFY);
      m_ota_characteristic->setCallbacks(&m_ota_callback);
    }
    {
      m_control_characteristic = service->createCharacteristic(constants::bluetooth::CONTROL_CHARACTERISTIC_UUID, WRITE);
      m_control_characteristic->setCallbacks(&m_control_callback);
    }
    {
      m_telemetry_characteristic = service->createCharacteristic(constants::bluetooth::TELEMETRY_CHARACTERISTIC_UUID, READ | NOTIFY);
    }

    NimBLEAdvertising* advertising = m_server->getAdvertising();
    advertising->setName(constants::bluetooth::DEVICE_NAME);
    advertising->addServiceUUID(service->getUUID());
    advertising->enableScanResponse(true);
    advertising->setMinInterval(32);  // 32 * 0.625ms = 20ms
    advertising->setMaxInterval(64);  // 64 * 0.625ms = 40ms
    advertising->start();

    return type::SystemError::None;
  }

  [[nodiscard]] auto isConnected() const -> bool { return m_server_callback.isConnected(); }

  [[nodiscard]] auto send_telemetry(type::SystemTelemetry const& data) const -> type::SystemError {
    if (!isConnected())
      return type::SystemError::BluetoothConnectedFault;

    if (m_telemetry_characteristic == nullptr)
      return type::SystemError::BluetoothInitFault;

    m_telemetry_characteristic->setValue(reinterpret_cast<std::uint8_t const*>(&data), sizeof(type::SystemTelemetry));

    std::ignore = m_telemetry_characteristic->notify();

    return type::SystemError::None;
  }

  [[nodiscard]] auto send_ota_notify(type::OTAStatus const status) const -> type::SystemError {
    if (!isConnected())
      return type::SystemError::BluetoothConnectedFault;

    if (m_ota_characteristic == nullptr)
      return type::SystemError::BluetoothInitFault;

    m_ota_characteristic->setValue(reinterpret_cast<std::uint8_t const*>(&status), sizeof(type::SystemTelemetry));
    std::ignore = m_ota_characteristic->notify();

    return type::SystemError::None;
  }
};

}  // namespace bluetooth
