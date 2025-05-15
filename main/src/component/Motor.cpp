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

#include "component/Motor.hpp"

#include <utility>

#include <esp_log.h>

namespace {

auto const TAG = "Motor";

} // namespace

auto Motor::create() -> std::expected<Motor::Pointer, Motor::Error> {
  auto encoder = foc::AS5600::create(21, 22, 0);
  if (not encoder) {
    return std::unexpected(Motor::Error::ENCODER_INIT_FAILED);
  }

  auto motor = std::make_unique<foc::Motor>(22, 0.2, 360, 20);
  if (not motor) {
    return std::unexpected(Motor::Error::MOTOR_INIT_FAILED);
  }

  auto driver = std::make_unique<foc::Driver6PWM>(14, 15, 22, 23, 31, 32);
  if (not driver) {
    return std::unexpected(Motor::Error::DRIVER_INIT_FAILED);
  }

  return Motor::Pointer(new Motor(std::move(*encoder), std::move(motor), std::move(driver)));
}

Motor::Motor(Motor::Encoder encoder, BLDCMotor motor, BLDCDriver driver)
    : m_encoder(std::move(encoder)), m_motor(std::move(motor)), m_driver(std::move(driver)) {

  m_motor->linkDriver(std::move(m_driver));
  m_motor->linkEncoder(std::move(m_encoder));

  m_motor->init();
  m_motor->initFOC();
}

void Motor::setPosition(Motor::Position position) { m_motor->move(position); }

void Motor::process() { m_motor->loopFOC(); }
