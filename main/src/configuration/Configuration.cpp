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

using Error = esp_err_t;
using Handle = nvs_handle_t;

auto nvs_set_float(Handle handle, char const *key, float value) -> Error {
  std::uint32_t buf = 0;

  std::memcpy(&buf, &value, sizeof(float));

  return nvs_set_u32(handle, key, buf);
}

auto nvs_get_float(Handle handle, char const *key, float *value) -> esp_err_t {
  std::uint32_t buf = 0;

  Error const err = nvs_get_u32(handle, key, &buf);
  if (err == ESP_OK) {
    std::memcpy(value, &buf, sizeof(float));
  }

  return err;
}

} // namespace

Configuration::Configuration() {
  Error err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &storageHandle));
}

Configuration::~Configuration() noexcept {
  nvs_commit(storageHandle);
  nvs_close(storageHandle);
}

auto Configuration::getIndicatorPin() const -> interface::Configuration::Pin { return CONFIG_INDICATOR_PIN; }
auto Configuration::getModeButtonPin() const -> interface::Configuration::Pin { return CONFIG_MODE_SWITCH_PIN; }
auto Configuration::getBreakSwitchPin() const -> interface::Configuration::Pin { return CONFIG_BREAK_SWITCH_PIN; }
auto Configuration::getGuardSwitchPin() const -> interface::Configuration::Pin { return CONFIG_GUARD_SWITCH_PIN; }
auto Configuration::getClutchSwitchPin() const -> interface::Configuration::Pin { return CONFIG_CLUTCH_SWITCH_PIN; }
auto Configuration::getTwistSensor1Pin() const -> interface::Configuration::Pin { return CONFIG_TWIST_SENSOR_1_PIN; }
auto Configuration::getTwistSensor2Pin() const -> interface::Configuration::Pin { return CONFIG_TWIST_SENSOR_2_PIN; }
