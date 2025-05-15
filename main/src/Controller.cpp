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

#include "Controller.hpp"

#include <utility>

#include <esp_log.h>
#include <esp_timer.h>

#include "component/TwistPosition.hpp"

namespace {

auto const TAG = "Controller";
auto const MICROSECONDS_IN_SECOND = 1000000;
auto const DEAD_TIME = 5 * MICROSECONDS_IN_SECOND;
auto const SPEED_MODE_CRUISE = 60;
auto const POSITION_MINIMAL = 0;
auto const POSITION_MAXIMAL = 100;

} // namespace

auto Controller::create(Configuration::Pointer configuration, TelemetryPointer telemetry) -> std::expected<Pointer, Error> {
  auto twistPosition = TwistPosition::create();
  if (not twistPosition) {
    return std::unexpected(Controller::Error::TWIST_POSITION_INIT_FAILED);
  }

  return Controller::Pointer(new Controller(std::move(configuration), std::move(telemetry), std::move(*twistPosition)));
}

Controller::Controller(Configuration::Pointer configuration, TelemetryPointer telemetry, TwistPosition::Pointer twistPosition)
    : m_executor(std::make_unique<executor::Executor>()), m_pid(std::make_unique<pid::PIDController>(5, 0, 0, 10, 1)), m_telemetry(std::move(telemetry)),
      m_configuration(std::move(configuration)), m_twistPosition(std::move(twistPosition)) {

  m_pid->reset();

  m_executor->addNode(m_twistPosition, 1);
  m_executor->start();
}

auto Controller::run() -> void {}

void Controller::process() {
  auto const currentTwistPosition = m_twistPosition->getPosition();

//  m_telemetry->setTwistPosition(currentTwistPosition);

  ESP_LOGI(TAG, "Twist position: %u", currentTwistPosition);

  //  auto throttlePosition = m_pid.compute(m_speed);
  //
  //  if (throttlePosition < m_twistPosition) {
  //    throttlePosition = m_twistPosition;
  //  }
  //
  //  if (throttlePosition < m_throttlePositionMinimal) {
  //    throttlePosition = m_throttlePositionMinimal;
  //  }
  //
  //  if (throttlePosition > m_throttlePositionMaximal) {
  //    throttlePosition = m_throttlePositionMaximal;
  //  }

  //  if (m_positionUpdateCallback) {
  //    m_positionUpdateCallback(throttlePosition);
  //  }
}
