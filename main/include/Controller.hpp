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

#include "PIDController.hpp"
#include "configuration/interface/Configuration.hpp"

#include <executor/Node.hpp>

class Controller : public executor::Node {
public:
  using RPM = std::uint16_t;
  using Time = std::int64_t;
  using Temp = float;
  using Speed = std::uint16_t;
  using Position = std::uint8_t;
  using ErrorCallback = std::function<void()>;
  using PIDController = PIDController<Controller::Position>;
  using CruiseEnabledCallback = std::function<void(bool)>;
  using PositionUpdateCallback = std::function<void(Controller::Position)>;

public:
  explicit Controller(ConfigurationPtr configuration);
  ~Controller() override = default;

public:
  void registerErrorCallback(Controller::ErrorCallback callback);
  void registerCruiseEnableCallback(Controller::CruiseEnabledCallback callback);
  void registerPositionUpdateCallback(Controller::PositionUpdateCallback callback);

public:
  void setRPM(Controller::RPM rpm);
  void setSpeed(Controller::Speed speed);
  void setTemperature(Controller::Temp temp);
  void setTwistPosition(Controller::Position position);
  void setThrottlePosition(Controller::Position position);

public:
  void enableGuardMode();

public:
  void holdRPM();
  void releaseRPM();

public:
  void enableCruiseMode();
  void disableCruiseMode();

private:
  void process() override;

private:
  ConfigurationPtr m_configuration = nullptr;
  Controller::ErrorCallback m_errorCallback = nullptr;
  Controller::CruiseEnabledCallback m_cruiseEnabledCallback = nullptr;
  Controller::PositionUpdateCallback m_positionUpdateCallback = nullptr;

private:
  Controller::PIDController m_pid;

private:
  Controller::RPM m_rpm = 0;
  Controller::Temp m_temp = 0;
  Controller::Speed m_speed = 0;
  Controller::Time m_rpmLastUpdate = 0;
  Controller::Time m_speedLastUpdate = 0;
  Controller::Position m_twistPosition = 0;
  Controller::Position m_throttlePosition = 0;
  Controller::Position m_throttlePositionMaximal = 0;
  Controller::Position m_throttlePositionMinimal = 0;
};
