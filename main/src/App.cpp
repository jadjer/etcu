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

#include "configuration/Configuration.hpp"

auto const TAG = "App";

App::App()
    : m_configuration(std::make_shared<Configuration>()), m_bluetooth(m_configuration),

      m_motor(std::make_unique<Motor>())

//             m_guardSwitch(std::make_unique<Switch>(m_configuration->getGuardSwitchPin())),
//             m_breakSwitch(std::make_unique<Switch>(m_configuration->getBreakSwitchPin())),
//             m_clutchSwitch(std::make_unique<Switch>(m_configuration->getClutchSwitchPin())),
//             m_throttle(std::make_unique<Throttle>()),
//             m_indicator(std::make_unique<Indicator>(m_configuration->getIndicatorPin())),
//             m_controller(std::make_unique<Controller>(m_configuration)),
//             m_modeButton(std::make_unique<ModeButton>(m_configuration->getModeButtonPin())),
//             m_twistPosition(std::make_unique<TwistPosition>())
{}

void App::setup() {
  //  m_guardSwitch->registerSwitchCallback([this](bool const isEnabled) {
  //    m_controller->enableGuardMode();
  //  });
  //  m_breakSwitch->registerSwitchCallback([this](bool const isEnabled) {
  //    if (isEnabled) {
  //      m_controller->disableCruiseMode();
  //    }
  //  });
  //  m_clutchSwitch->registerSwitchCallback([this](bool const isEnabled) {
  //    if (isEnabled) {
  //      m_controller->disableCruiseMode();
  //    }
  //  });
  //
  //  m_controller->registerPositionUpdateCallback([this](Controller::Position const position) {
  //    m_throttle->setPosition(position);
  //    m_controller->setThrottlePosition(position);
  //  });
  //  m_controller->registerCruiseEnableCallback([this](bool const isEnabled) {
  //    if (isEnabled) {
  //      m_indicator->setMode(Indicator::CRUISE_ON_MODE);
  //    } else {
  //      m_indicator->setMode(Indicator::NORMAL_MODE);
  //    }
  //  });
  //  m_controller->registerErrorCallback([this] {
  //    m_indicator->setError();
  //  });
  //
  //  m_modeButton->registerHoldCallback([this] {
  //    m_controller->holdRPM();
  //    m_controller->enableCruiseMode();
  //  });
  //  m_modeButton->registerPressCallback([this] {
  //    m_controller->releaseRPM();
  //    m_controller->disableCruiseMode();
  //  });
  //
  //  m_twistPosition->registerPositionChangeCallback([this](TwistPosition::Position const position) {
  //    m_controller->setTwistPosition(position);
  //  });
  //  m_twistPosition->registerErrorCallback([this] {
  //    m_indicator->setError();
  //  });
}

void App::run() {
  m_bluetooth.advertise();

  m_executor.addNode(m_motor);
  //    m_executor.addNode(m_guardSwitch);
  //  m_executor.addNode(m_breakSwitch);
  //  m_executor.addNode(m_clutchSwitch);
  //  m_executor.addNode(m_throttle);
  //  m_executor.addNode(m_indicator);
  //  m_executor.addNode(m_controller);
  //  m_executor.addNode(m_modeButton);
  //  m_executor.addNode(m_twistPosition);

  m_executor.spin();
}
