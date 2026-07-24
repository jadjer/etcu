//
// Created by jadjer on 25.07.26.
//

#pragma once

#include <nvs.h>
#include "concepts/concepts.hpp"

class Storage {
  static constexpr auto NvsNamespace = "accel_cali";
  static constexpr auto NvsBlobKey = "cali_blob";

 public:
  template <concepts::HasStructVersion T>
  [[nodiscard]] auto load_calibration(T &data) const noexcept -> bool {
    nvs_handle_t nvs_handle = 0;

    if (nvs_open(NvsNamespace, NVS_READONLY, &nvs_handle) != ESP_OK) {
      return false;
    }

    std::size_t requiredSize = sizeof(T);

    esp_err_t const err = nvs_get_blob(nvs_handle, NvsBlobKey, &data, &requiredSize);

    nvs_close(nvs_handle);

    if (err != ESP_OK) {
      return false;
    }

    return data.struct_version == T{}.struct_version;
  }

  template <concepts::HasStructVersion T>
  [[nodiscard]] auto save_calibration(T data) const noexcept -> bool {
    nvs_handle_t nvs_handle = 0;

    if (nvs_open(NvsNamespace, NVS_READWRITE, &nvs_handle) != ESP_OK) {
      return false;
    }

    esp_err_t err = nvs_set_blob(nvs_handle, NvsBlobKey, &data, sizeof(T));

    if (err == ESP_OK) {
      err = nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);

    return (err == ESP_OK);
  }
};
