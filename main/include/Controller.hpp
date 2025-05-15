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

#include <cstdint>
#include <functional>
#include <memory>

#include <executor/Executor.hpp>
#include <executor/Node.hpp>
#include <pid/PIDController.hpp>

#include "Configuration.hpp"
#include "Telemetry.hpp"
#include "component/TwistPosition.hpp"

class Controller : public executor::Node {
public:
  enum class Error : std::uint8_t {
    TWIST_POSITION_INIT_FAILED,
  };

public:
  using RPM = std::uint16_t;
  using Time = std::int64_t;
  using Temp = float;
  using Speed = std::uint16_t;
  using Pointer = std::unique_ptr<Controller>;
  using Position = std::uint8_t;
  using ExecutorPointer = std::unique_ptr<executor::Executor>;
  using TelemetryPointer = std::shared_ptr<Telemetry>;
  using PIDControllerPointer = std::unique_ptr<pid::PIDController>;

public:
  static auto create(Configuration::Pointer configuration, TelemetryPointer telemetry) -> std::expected<Pointer, Error>;

private:
  Controller(Configuration::Pointer configuration, TelemetryPointer telemetry, TwistPosition::Pointer twistPosition);

public:
  ~Controller() override = default;

public:
  auto run() -> void;

private:
  void process() override;

private:
  ExecutorPointer const m_executor;
  PIDControllerPointer const m_pid;
  TelemetryPointer const m_telemetry;
  Configuration::Pointer const m_configuration;
  TwistPosition::Pointer const m_twistPosition;
};
