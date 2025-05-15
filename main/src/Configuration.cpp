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

#include "Configuration.hpp"

#include <cstring>

#include <nvs.h>
#include <nvs_flash.h>

#include "sdkconfig.h"

namespace {

using Result = esp_err_t;
using Handle = nvs_handle_t;

auto nvs_set_float(Handle handle, char const *key, float value) -> Result {
  std::uint32_t buf = 0;

  std::memcpy(&buf, &value, sizeof(float));

  return nvs_set_u32(handle, key, buf);
}

auto nvs_get_float(Handle handle, char const *key, float *value) -> Result {
  std::uint32_t buf = 0;

  Result const err = nvs_get_u32(handle, key, &buf);
  if (err == ESP_OK) {
    std::memcpy(value, &buf, sizeof(float));
  }

  return err;
}

} // namespace

auto Configuration::create() -> std::expected<Configuration::Pointer, Configuration::Error> {
  Result initResult = nvs_flash_init();
  if (initResult == ESP_ERR_NVS_NO_FREE_PAGES || initResult == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    Result eraseResult = nvs_flash_erase();
    if (eraseResult != ESP_OK) {
      return std::unexpected(Configuration::Error::CONFIGURATION_ERASE_STORAGE_ERROR);
    }

    initResult = nvs_flash_init();
  }
  if (initResult != ESP_OK) {
    return std::unexpected(Configuration::Error::CONFIGURATION_INIT_STORAGE_ERROR);
  }

  Configuration::StorageHandle storageHandle;

  Result openResult = nvs_open("storage", NVS_READWRITE, &storageHandle);
  if (openResult != ESP_OK) {
    return std::unexpected(Configuration::Error::CONFIGURATION_OPEN_STORAGE_ERROR);
  }

  return Configuration::Pointer(new Configuration(std::move(storageHandle)));
}

Configuration::Configuration(StorageHandle storageHandle) noexcept : m_storageHandle(std::move(storageHandle)) {}

Configuration::~Configuration() noexcept {
  nvs_commit(m_storageHandle);
  nvs_close(m_storageHandle);
}

auto Configuration::getIndicatorPin() const -> Configuration::Pin { return CONFIG_INDICATOR_PIN; }
auto Configuration::getModeButtonPin() const -> Configuration::Pin { return CONFIG_MODE_SWITCH_PIN; }
auto Configuration::getBreakSwitchPin() const -> Configuration::Pin { return CONFIG_BREAK_SWITCH_PIN; }
auto Configuration::getGuardSwitchPin() const -> Configuration::Pin { return CONFIG_GUARD_SWITCH_PIN; }
auto Configuration::getClutchSwitchPin() const -> Configuration::Pin { return CONFIG_CLUTCH_SWITCH_PIN; }
auto Configuration::getTwistSensor1Pin() const -> Configuration::Pin { return CONFIG_TWIST_SENSOR_1_PIN; }
auto Configuration::getTwistSensor2Pin() const -> Configuration::Pin { return CONFIG_TWIST_SENSOR_2_PIN; }
