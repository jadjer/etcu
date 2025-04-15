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

#include "Controller.hpp"
#include "HourMeter.hpp"
#include "Indicator.hpp"
#include "ModeButton.hpp"
#include "Motor.hpp"
#include "Switch.hpp"
#include "Throttle.hpp"
#include "TwistPosition.hpp"
#include "bluetooth/Bluetooth.hpp"
#include "configuration/interface/Configuration.hpp"

#include <executor/Executor.hpp>
#include <memory>

class App {
public:
  using MotorPtr = std::shared_ptr<Motor>;
  using SwitchPtr = std::shared_ptr<Switch>;
  using ThrottlePtr = std::shared_ptr<Throttle>;
  using IndicatorPtr = std::shared_ptr<Indicator>;
  using ControllerPtr = std::shared_ptr<Controller>;
  using ModeButtonPtr = std::shared_ptr<ModeButton>;
  using TwistPositionPtr = std::shared_ptr<TwistPosition>;

public:
  App();

public:
  void setup();

  void run();

private:
  ConfigurationPtr const m_configuration = nullptr;

private:
  Bluetooth m_bluetooth;
  HourMeter m_hourMeter;
  executor::Executor m_executor;

private:
  App::MotorPtr m_motor = nullptr;
  App::SwitchPtr m_guardSwitch = nullptr;
  App::SwitchPtr m_breakSwitch = nullptr;
  App::SwitchPtr m_clutchSwitch = nullptr;
  App::ThrottlePtr m_throttle = nullptr;
  App::IndicatorPtr m_indicator = nullptr;
  App::ControllerPtr m_controller = nullptr;
  App::ModeButtonPtr m_modeButton = nullptr;
  App::TwistPositionPtr m_twistPosition = nullptr;
};
