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
#include <memory>

#include <executor/Executor.hpp>

#include "Configuration.hpp"
#include "Telemetry.hpp"
#include "bluetooth/Bluetooth.hpp"

class App {
public:
  enum class Error : std::uint8_t {
    CONFIGURATION_ERROR,
    INDICATOR_ERROR,
    TWIST_POSITION_ERROR,
    MOTOR_ERROR,
    CONTROLLER_INIT_ERROR,
  };

public:
  using Pointer = std::unique_ptr<App>;
  using ExecutorPointer = std::unique_ptr<executor::Executor>;
  using BluetoothPointer = std::unique_ptr<Bluetooth>;
  using TelemetryPointer = std::shared_ptr<Telemetry>;
  using ConfigurationPointer = std::shared_ptr<Configuration>;

public:
  static auto create() -> std::expected<Pointer, Error>;

private:
  explicit App(Configuration::Pointer const &configuration) noexcept;

public:
  auto setup() -> std::expected<void, Error>;
  auto run() -> void;

private:
  ExecutorPointer const executor;
  TelemetryPointer const telemetry;
  BluetoothPointer const bluetooth;
  ConfigurationPointer const configuration;
};
