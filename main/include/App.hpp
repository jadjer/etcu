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
#include <source_location>
#include <string_view>
#include <utility>

#include <executor/Executor.hpp>

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

class App {
public:
  using MotorPointer = std::shared_ptr<Motor>;
  using SwitchPointer = std::shared_ptr<Switch>;
  using ThrottlePointer = std::shared_ptr<Throttle>;
  using IndicatorPointer = std::shared_ptr<Indicator>;
  using ControllerPointer = std::shared_ptr<Controller>;
  using ModeButtonPointer = std::shared_ptr<ModeButton>;
  using TwistPositionPointer = std::unique_ptr<TwistPosition>;

private:
  struct Components {
    MotorPointer motor;
    SwitchPointer switchGuard;
    SwitchPointer switchBreak;
    SwitchPointer switchClutch;
    ThrottlePointer throttle;
    IndicatorPointer indicator;
    ControllerPointer controller;
    ModeButtonPointer modeButton;
    TwistPositionPointer twistPosition;
  };

public:
  explicit App(Bluetooth::DeviceName const& deviceName);

public:
  void setup();
  void run();

private:
  ConfigurationSharedPtr const configurationPointer = nullptr;

private:
  Bluetooth bluetooth;
  HourMeter hourMeter;
  Components components;
  executor::Executor executor;
};
