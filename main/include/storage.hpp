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
// Created by jadjer on 25.07.26.
//

#pragma once

#include <nvs.h>
#include "concepts.hpp"
#include "constants.hpp"

class Storage {
 public:

  template <class T>
    requires concepts::HasStructVersion<T>
  [[nodiscard]] auto load_calibration(T& data) const noexcept -> bool {
    nvs_handle_t nvs_handle = 0;

    if (nvs_open(constants::system::NVS_NAMESPACE, NVS_READONLY, &nvs_handle) != ESP_OK)
      return false;

    std::size_t requiredSize = sizeof(T);

    esp_err_t const err = nvs_get_blob(nvs_handle, T::StructName, &data, &requiredSize);

    nvs_close(nvs_handle);

    if (err != ESP_OK || requiredSize != sizeof(T))
      return false;

    return data.struct_version == T{}.struct_version;
  }

  template <class T>
    requires concepts::HasStructVersion<T>
  [[nodiscard]] auto save_calibration(const T& data) const noexcept -> bool {
    nvs_handle_t nvs_handle = 0;

    if (nvs_open(constants::system::NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) != ESP_OK)
      return false;

    esp_err_t err = nvs_set_blob(nvs_handle, T::StructName, &data, sizeof(T));

    if (err == ESP_OK)
      err = nvs_commit(nvs_handle);

    nvs_close(nvs_handle);

    return err == ESP_OK;
  }
};
