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
#include <nvs_flash.h>
#include <nvs_handle.hpp>

#include "config/concepts.hpp"
#include "config/constants.hpp"

template <typename T>
concept HasStructVersionConcept = std::is_trivially_copyable_v<T> && requires(T instance) {
  { T::current_version } -> std::convertible_to<std::uint32_t>;
  { T::name } -> std::convertible_to<std::string_view>;

  { instance.version } -> std::same_as<std::uint32_t&>;
};

class Storage {
  nvs_handle_t m_handle{0};

 public:
  constexpr Storage() noexcept = default;

  Storage(Storage const&) noexcept = delete;
  auto operator=(Storage const&) noexcept -> Storage& = delete;

  Storage(Storage&&) noexcept = delete;
  auto operator=(Storage&&) noexcept -> Storage& = delete;

  constexpr ~Storage() noexcept = default;

  auto init() noexcept -> bool {
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      if (nvs_flash_erase() != ESP_OK) [[unlikely]] {
        return false;
      }

      error = nvs_flash_init();
    }

    if (error != ESP_OK) [[unlikely]] {
      return false;
    }

    if (nvs_open(constants::system::Name.data(), NVS_READWRITE, &m_handle) != ESP_OK) {
      return false;
    }

    return true;
  }

  template <class T>
    requires HasStructVersionConcept<T>
  auto load(T& data) const noexcept -> bool {
    static constexpr std::size_t data_size{sizeof(T)};
    static constexpr std::uint32_t data_version{T::current_version};
    static constexpr std::string_view data_name{T::name};

    std::size_t required_size{data_size};

    if (esp_err_t const error = nvs_get_blob(m_handle, data_name.data(), &data, &required_size); error != ESP_OK) [[unlikely]] {
      return false;
    }

    if (required_size != data_size) [[unlikely]] {
      return false;
    }

    if (data.version != data_version) [[unlikely]] {
      return false;
    }

    return true;
  }

  template <class T>
    requires HasStructVersionConcept<T>
  auto save(T const& data) const noexcept -> bool {
    static constexpr std::size_t data_size{sizeof(T)};
    static constexpr std::uint32_t data_version{T::current_version};
    static constexpr std::string_view data_name{T::name};

    T local_data = data;

    local_data.version = data_version;

    if (nvs_set_blob(m_handle, data_name.data(), &local_data, data_size) != ESP_OK) [[unlikely]] {
      return false;
    }

    if (nvs_commit(m_handle) != ESP_OK) [[unlikely]] {
      return false;
    }

    return true;
  }
};
