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
#include "commons/atomic_channel.hpp"
#include "constants.hpp"
#include "types.hpp"

namespace bluetooth {

class BLEManager {
  NimBLEServer* m_server{nullptr};
  NimBLECharacteristic* m_ota_characteristic{nullptr};
  NimBLECharacteristic* m_control_characteristic{nullptr};
  NimBLECharacteristic* m_telemetry_characteristic{nullptr};

  callbacks::ServerCallback m_server_callback;
  callbacks::CharacteristicCallback<OTAChunk> m_ota_callback;
  callbacks::CharacteristicCallback<BluetoothControl> m_control_callback;

 public:
  explicit BLEManager(commons::AtomicChannel<OTAChunk>& ota_chunk, commons::AtomicChannel<BluetoothControl>& control)
      : m_ota_callback(ota_chunk), m_control_callback(control) {}

  auto init() -> SystemError {
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    if (!NimBLEDevice::init(DEVICE_NAME)) {
      return SystemError::BluetoothInitFault;
    }

    if (!NimBLEDevice::setPower(ESP_PWR_LVL_P9)) {
      return SystemError::BluetoothInitFault | SystemError::BluetoothSetPowerFault;
    }

    if (!NimBLEDevice::setMTU(512)) {
      return SystemError::BluetoothInitFault | SystemError::BluetoothSetMTUFault;
    }

    m_server = NimBLEDevice::createServer();
    m_server->setCallbacks(&m_server_callback);

    NimBLEService* service = m_server->createService(SERVICE_UUID);
    {
      m_ota_characteristic = service->createCharacteristic(OTA_CHARACTERISTIC_UUID, WRITE | NOTIFY);
      m_ota_characteristic->setCallbacks(&m_ota_callback);
    }
    {
      m_control_characteristic = service->createCharacteristic(CONTROL_CHARACTERISTIC_UUID, WRITE);
      m_control_characteristic->setCallbacks(&m_control_callback);
    }
    {
      m_telemetry_characteristic = service->createCharacteristic(TELEMETRY_CHARACTERISTIC_UUID, READ | NOTIFY);
    }

    NimBLEAdvertising* advertising = m_server->getAdvertising();
    advertising->setName(DEVICE_NAME);
    advertising->addServiceUUID(service->getUUID());
    advertising->enableScanResponse(true);
    advertising->setMinInterval(32);  // 32 * 0.625ms = 20ms
    advertising->setMaxInterval(64);  // 64 * 0.625ms = 40ms
    advertising->start();

    return SystemError::None;
  }

  [[nodiscard]] auto isConnected() const -> bool { return m_server_callback.isConnected(); }

  [[nodiscard]] auto send_telemetry(SystemTelemetry const data) const -> SystemError {
    if (!isConnected())
      return SystemError::BluetoothConnectedFault;

    if (m_telemetry_characteristic == nullptr)
      return SystemError::BluetoothInitFault;

    m_telemetry_characteristic->setValue(reinterpret_cast<std::uint8_t const*>(&data), sizeof(SystemTelemetry));

    std::ignore = m_telemetry_characteristic->notify();

    return SystemError::None;
  }

  [[nodiscard]] auto send_ota_notify(OTAStatus const status) const -> SystemError {
    if (!isConnected())
      return SystemError::BluetoothConnectedFault;

    if (m_ota_characteristic == nullptr)
      return SystemError::BluetoothInitFault;

    m_ota_characteristic->setValue(reinterpret_cast<std::uint8_t const*>(&status), sizeof(SystemTelemetry));
    std::ignore = m_ota_characteristic->notify();

    return SystemError::None;
  }
};

}  // namespace bluetooth
