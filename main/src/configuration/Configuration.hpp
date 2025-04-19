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

#include <nvs_handle.hpp>

#include "configuration/interface/Configuration.hpp"

class Configuration : public interface::Configuration {
private:
  using StorageHandle = nvs_handle_t;

public:
  Configuration();
  ~Configuration() noexcept override;

public:
  [[nodiscard]] [[maybe_unused]] auto getIndicatorPin() const -> interface::Configuration::Pin override;
  [[nodiscard]] [[maybe_unused]] auto getModeButtonPin() const -> interface::Configuration::Pin override;
  [[nodiscard]] [[maybe_unused]] auto getBreakSwitchPin() const -> interface::Configuration::Pin override;
  [[nodiscard]] [[maybe_unused]] auto getGuardSwitchPin() const -> interface::Configuration::Pin override;
  [[nodiscard]] [[maybe_unused]] auto getClutchSwitchPin() const -> interface::Configuration::Pin override;
  [[nodiscard]] [[maybe_unused]] auto getTwistSensor1Pin() const -> interface::Configuration::Pin override;
  [[nodiscard]] [[maybe_unused]] auto getTwistSensor2Pin() const -> interface::Configuration::Pin override;

private:
  StorageHandle storageHandle{0};
};
