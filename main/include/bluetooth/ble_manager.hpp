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
#include "bluetooth/callbacks/ota_callback.hpp"
#include "bluetooth/callbacks/server_callback.hpp"
#include "bluetooth/callbacks/system_info_callback.hpp"
#include "callbacks/control_callback.hpp"
#include "common/atomic_container.hpp"
#include "config/constants.hpp"
#include "type/telemetry.hpp"

namespace bluetooth {

class BLEManager {
  NimBLEServer* m_server{nullptr};
  NimBLECharacteristic* m_ota_characteristic{nullptr};
  NimBLECharacteristic* m_control_characteristic{nullptr};
  NimBLECharacteristic* m_telemetry_characteristic{nullptr};
  NimBLECharacteristic* m_system_info_characteristic{nullptr};

  callback::OTACallback m_ota_callback;
  callback::ServerCallback m_server_callback{};
  callback::ControlCallback m_control_callback;
  callback::SystemInfoCallback m_system_info_callback{};

 public:
  constexpr explicit BLEManager(common::AtomicContainer<type::Control>& control,
                                common::AtomicContainer<type::OTAChunk<constants::bluetooth::OTAPayloadSize>>& ota_chunk)
      : m_ota_callback(ota_chunk), m_control_callback(control) {}

  constexpr BLEManager() noexcept = delete;

  BLEManager(BLEManager const&) noexcept = delete;
  auto operator=(BLEManager const&) noexcept -> BLEManager& = delete;

  BLEManager(BLEManager&&) noexcept = delete;
  auto operator=(BLEManager&&) noexcept -> BLEManager& = delete;

  ~BLEManager() noexcept = default;

  [[nodiscard]] constexpr auto init() -> type::SystemError {
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    if (!NimBLEDevice::init(constants::bluetooth::DeviceName.data()))
      return type::SystemError::BluetoothInitFault;

    if (!NimBLEDevice::setPower(ESP_PWR_LVL_N24))
      return type::SystemError::BluetoothInitFault | type::SystemError::BluetoothSetPowerFault;

    if (!NimBLEDevice::setMTU(517))
      return type::SystemError::BluetoothInitFault | type::SystemError::BluetoothSetMTUFault;

    m_server = NimBLEDevice::createServer();
    m_server->setCallbacks(&m_server_callback);

    NimBLEService* service = m_server->createService(constants::bluetooth::ServiceUUID.data());
    m_ota_characteristic = service->createCharacteristic(constants::bluetooth::OTACharUUID.data(), WRITE | NOTIFY);
    m_control_characteristic = service->createCharacteristic(constants::bluetooth::ControlCharUUID.data(), READ | WRITE);
    m_telemetry_characteristic = service->createCharacteristic(constants::bluetooth::TelemetryCharUUID.data(), READ | NOTIFY);
    m_system_info_characteristic = service->createCharacteristic(constants::bluetooth::SysInfoCharUUID.data(), READ);

    m_ota_characteristic->setCallbacks(&m_ota_callback);
    m_control_characteristic->setCallbacks(&m_control_callback);
    m_system_info_characteristic->setCallbacks(&m_system_info_callback);

    NimBLEAdvertising* advertising = m_server->getAdvertising();
    advertising->setName(constants::bluetooth::DeviceName.data());
    advertising->addServiceUUID(service->getUUID());
    advertising->enableScanResponse(true);
    advertising->setMinInterval(32);
    advertising->setMaxInterval(64);
    advertising->start();

    return type::SystemError::None;
  }

  [[nodiscard]] auto isConnected() const -> bool { return m_server_callback.isConnected(); }

  [[nodiscard]] auto send_telemetry(type::SystemTelemetry const& data) const -> type::SystemError {
    if (!isConnected())
      return type::SystemError::BluetoothConnectedFault;

    if (m_telemetry_characteristic == nullptr)
      return type::SystemError::BluetoothInitFault;

    type::dto::SystemTelemetry const system_telemetry = data.to_dto();

    m_telemetry_characteristic->setValue(system_telemetry);
    std::ignore = m_telemetry_characteristic->notify();

    return type::SystemError::None;
  }

  [[nodiscard]] auto send_ota_notify(type::OTAStatus const status) const -> type::SystemError {
    if (!isConnected())
      return type::SystemError::BluetoothConnectedFault;

    if (m_ota_characteristic == nullptr)
      return type::SystemError::BluetoothInitFault;

    std::ignore = m_ota_characteristic->notify(status);

    return type::SystemError::None;
  }
};

}  // namespace bluetooth
