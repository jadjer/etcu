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

#include "component/TwistPosition.hpp"

#include <utility>

namespace {

auto const TAG = "Twist Position";

template <typename T>
  requires std::is_arithmetic_v<T>
[[nodiscard]] auto convertVoltageToPercentage(adc::Channel::Voltage value, T fromMin, T fromMax) -> T {
  if (fromMin == fromMax) {
    fromMin -= 1;
    fromMax += 1;
  }

  T const fromRange = fromMax - fromMin;
  T const scaledValue = (value - fromMin) / fromRange;

  T result = scaledValue * 100.0;

  if (result < 0.0) {
    result = 0.0;
  }
  if (result > 100.0) {
    result = 100.0;
  }

  return result;
}

} // namespace

auto TwistPosition::create() -> std::expected<TwistPosition::Pointer, TwistPosition::Error> {
  auto unit = adc::Unit::create(0);
  if (not unit) {
    return std::unexpected(TwistPosition::Error::ADC_INIT_FAILED);
  }

  auto sensor1 = (*unit)->createChannel(3);
  if (not sensor1) {
    return std::unexpected(TwistPosition::Error::SENSOR1_INIT_FAILED);
  }

  auto sensor2 = (*unit)->createChannel(6);
  if (not sensor2) {
    return std::unexpected(TwistPosition::Error::SENSOR2_INIT_FAILED);
  }

  return TwistPosition::Pointer(new TwistPosition(std::move(*unit), std::move(*sensor1), std::move(*sensor2)));
}

TwistPosition::TwistPosition(adc::Unit::Pointer unit, adc::Channel::Pointer sensor1, adc::Channel::Pointer sensor2)
    : unit(std::move(unit)), sensor1(std::move(sensor1)), sensor2(std::move(sensor2)), m_positionMinSensor1(1000), m_positionMaxSensor1(3000), m_positionMinSensor2(500),
      m_positionMaxSensor2(1500) {}

auto TwistPosition::getPosition() const -> TwistPosition::Position { return m_position; }

auto TwistPosition::process() -> void {
  auto const voltageSensor1 = sensor1->getVoltage();
  auto const positionSensor1 = convertVoltageToPercentage(voltageSensor1, m_positionMinSensor1, m_positionMaxSensor1);

  auto const voltageSensor2 = sensor2->getVoltage();
  auto const positionSensor2 = convertVoltageToPercentage(voltageSensor2, m_positionMinSensor2, m_positionMaxSensor2);

  auto diffSensorValue = positionSensor1 - positionSensor2;
  if (diffSensorValue < 0) {
    diffSensorValue *= -1;
  }

  if (diffSensorValue > 5) {
    m_position = 0;
  } else {
    m_position = positionSensor1;
  }
}
