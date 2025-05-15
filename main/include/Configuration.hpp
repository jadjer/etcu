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

#pragma once

#include <expected>
#include <nvs_handle.hpp>

class Configuration {
public:
  enum class Error : std::uint8_t {
    CONFIGURATION_ERASE_STORAGE_ERROR,
    CONFIGURATION_INIT_STORAGE_ERROR,
    CONFIGURATION_OPEN_STORAGE_ERROR,
  };

public:
  using Pin = std::uint8_t;
  using Pointer = std::shared_ptr<Configuration>;

private:
  using StorageHandle = nvs_handle_t;

public:
  static auto create() -> std::expected<Pointer, Error>;

private:
  explicit Configuration(StorageHandle storageHandle) noexcept;

public:
  ~Configuration() noexcept;

public:
  [[nodiscard]] [[maybe_unused]] auto getIndicatorPin() const -> Pin;
  [[nodiscard]] [[maybe_unused]] auto getModeButtonPin() const -> Pin;
  [[nodiscard]] [[maybe_unused]] auto getBreakSwitchPin() const -> Pin;
  [[nodiscard]] [[maybe_unused]] auto getGuardSwitchPin() const -> Pin;
  [[nodiscard]] [[maybe_unused]] auto getClutchSwitchPin() const -> Pin;
  [[nodiscard]] [[maybe_unused]] auto getTwistSensor1Pin() const -> Pin;
  [[nodiscard]] [[maybe_unused]] auto getTwistSensor2Pin() const -> Pin;

private:
  StorageHandle m_storageHandle{0};
};
