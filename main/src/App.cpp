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

#include "App.hpp"

#include <utility>

#include "configuration/Configuration.hpp"

namespace {

auto const TAG = "App";

} // namespace

App::App(Bluetooth::DeviceName const& deviceName) try
    : configurationPointer(std::make_shared<Configuration>()), bluetooth(configurationPointer, deviceName),
      components(Components{
          .motor = std::make_shared<Motor>(),
          .switchGuard = std::make_shared<Switch>(configurationPointer->getGuardSwitchPin()),
          .switchBreak = std::make_shared<Switch>(configurationPointer->getBreakSwitchPin()),
          .switchClutch = std::make_shared<Switch>(configurationPointer->getClutchSwitchPin()),
          .throttle = std::make_shared<Throttle>(),
          .indicator = std::make_shared<Indicator>(configurationPointer->getIndicatorPin()),
          .controller = std::make_shared<Controller>(configurationPointer),
          .modeButton = std::make_shared<ModeButton>(configurationPointer->getModeButtonPin()),
          .twistPosition = std::make_unique<TwistPosition>(),
      }) {
} catch (std::exception const &e) {
  ESP_LOGE(TAG, "Init app error: %s", e.what());
  esp_restart();
}

void App::setup() {
  components.switchGuard->registerSwitchCallback([this]([[maybe_unused]] bool const isEnabled) { components.controller->enableGuardMode(); });
  components.switchBreak->registerSwitchCallback([this](bool const isEnabled) {
    if (isEnabled) {
      components.controller->disableCruiseMode();
    }
  });
  components.switchClutch->registerSwitchCallback([this](bool const isEnabled) {
    if (isEnabled) {
      components.controller->disableCruiseMode();
    }
  });

  components.controller->registerPositionUpdateCallback([this](Controller::Position const position) {
    components.throttle->setPosition(position);
    components.controller->setThrottlePosition(position);
  });
  components.controller->registerCruiseEnableCallback([this](bool const isEnabled) {
    if (isEnabled) {
      components.indicator->setMode(Indicator::CRUISE_ON_MODE);
    } else {
      components.indicator->setMode(Indicator::NORMAL_MODE);
    }
  });
  components.controller->registerErrorCallback([this] { components.indicator->setError(); });

  components.modeButton->registerHoldCallback([this] {
    components.controller->holdRPM();
    components.controller->enableCruiseMode();
  });
  components.modeButton->registerPressCallback([this] {
    components.controller->releaseRPM();
    components.controller->disableCruiseMode();
  });

  components.twistPosition->registerPositionChangeCallback([this](TwistPosition::Position const position) { components.controller->setTwistPosition(position); });
  components.twistPosition->registerErrorCallback([this] { components.indicator->setError(); });
}

void App::run() {
  bluetooth.advertise();

  executor.addNode(components.motor);
  executor.addNode(components.switchGuard);
  executor.addNode(components.switchBreak);
  executor.addNode(components.switchClutch);
  executor.addNode(components.throttle);
  executor.addNode(components.indicator);
  executor.addNode(components.controller);
  executor.addNode(components.modeButton);
  executor.addNode(std::move(components.twistPosition));

  executor.spin();
}
