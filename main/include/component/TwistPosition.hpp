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

#include <concepts>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>

#include <adc/Channel.hpp>
#include <adc/Unit.hpp>
#include <executor/Node.hpp>

class TwistPosition : public executor::Node {
public:
  enum class Error : std::uint8_t {
    ADC_INIT_FAILED,
    SENSOR1_INIT_FAILED,
    SENSOR2_INIT_FAILED,
  };

public:
  using Pointer = std::shared_ptr<TwistPosition>;
  using Position = std::uint8_t;

private:
  using SensorPosition = std::uint16_t;

public:
  static auto create() -> std::expected<Pointer, Error>;

private:
  TwistPosition(adc::Unit::Pointer unit, adc::Channel::Pointer sensor1, adc::Channel::Pointer sensor2);

public:
  [[nodiscard]] auto getPosition() const -> Position;

private:
  auto process() -> void override;

private:
  adc::Unit::Pointer const unit;
  adc::Channel::Pointer const sensor1;
  adc::Channel::Pointer const sensor2;

private:
  Position m_position{0};
  SensorPosition m_positionMinSensor1{0};
  SensorPosition m_positionMaxSensor1{0};
  SensorPosition m_positionMinSensor2{0};
  SensorPosition m_positionMaxSensor2{0};
};
