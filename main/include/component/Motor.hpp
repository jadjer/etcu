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
#include <memory>

#include <executor/Node.hpp>
#include <foc/Motor.hpp>
#include <foc/driver/Driver6PWM.hpp>
#include <foc/encoder/AS5600.hpp>

class Motor : public executor::Node {
public:
  enum class Error : std::uint8_t {
    ENCODER_INIT_FAILED,
    MOTOR_INIT_FAILED,
    DRIVER_INIT_FAILED,
  };

public:
  using Position = std::uint16_t;

public:
  using BLDCDriver = std::unique_ptr<foc::Driver6PWM>;
  using BLDCMotor = std::unique_ptr<foc::Motor>;
  using Encoder = foc::AS5600::Pointer;
  using Pointer = std::shared_ptr<Motor>;

public:
  static auto create() -> std::expected<Pointer, Error>;

private:
  Motor(Encoder encoder, BLDCMotor motor, BLDCDriver driver);

public:
  ~Motor() override = default;

public:
  void setPosition(Position position);

private:
  void process() override;

private:
  Encoder m_encoder{nullptr};
  BLDCMotor m_motor{nullptr};
  BLDCDriver m_driver{nullptr};

private:
  Motor::Position m_targetPosition = 0;
  Motor::Position m_currentPosition = 0;
};
