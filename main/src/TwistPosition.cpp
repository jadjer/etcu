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

#include "TwistPosition.hpp"

#include <esp_log.h>
#include <utility>

namespace {

auto const TAG = "Twist Position";

struct Range {
  float min;
  float max;
};

// Функция для перевода значения из одного диапазона в другой с возвратом std::uint8_t
[[nodiscard]] auto convertVoltageToPercentage(adc::Channel::Voltage value, Channel::Voltage fromMin, Range to = {.min = 0.f, .max = 100.f}) -> float {
  // Проверка на случай, если исходный диапазон нулевой
  if (from.min == from.max) {
    from.min -= 1;
    from.max += 1;
  }

  // Вычисление пропорции
  float const fromRange = from.max - from.min;
  float const toRange = to.max - to.min;

  // Перевод значения в новый диапазон
  float const scaledValue = (value - from.min) / fromRange;
  float result = to.min + (scaledValue * toRange);

  if (result < to.min) {
    result = to.min;
  }
  if (result > to.max) {
    result = to.max;
  }

  result += 0.5f;

  return static_cast<std::uint8_t>(result);
}

} // namespace

TwistPosition::TwistPosition() try : unit(std::make_unique<adc::Unit>(0)), positionMinSensor1(1000), positionMaxSensor1(3000), positionMinSensor2(500), positionMaxSensor2(1500) {
  sensor1 = std::make_unique<adc::Channel>(unit->createChannel(3));
  sensor2 = std::make_unique<adc::Channel>(unit->createChannel(6));

  //  sensor1->enableFilter();
  //  sensor2->enableFilter();

  ESP_LOGI(TAG, "TwistPosition initialized successfully");
} catch (std::exception const &e) {
  ESP_LOGE(TAG, "TwistPosition initialization failed: %s", e.what());
}

auto TwistPosition::registerErrorCallback(TwistPosition::ErrorCallback callback) -> void {
  if (not callback) {
    throw std::invalid_argument("Error callback cannot be null");
  }

  errorCallback = std::move(callback);
}

auto TwistPosition::registerPositionChangeCallback(TwistPosition::PositionChangeCallback callback) -> void { positionChangeCallback = std::move(callback); }

auto TwistPosition::getPosition() const -> TwistPosition::Position { return position; }

auto TwistPosition::process() -> void {
  auto const voltageSensor1 = sensor1->getVoltage();
  auto const positionSensor1 = convertPositionToPercentage(voltageSensor1, Range{.min = positionMinSensor1, .max = positionMaxSensor1});

  auto const voltageSensor2 = sensor2->getVoltage();
  auto const positionSensor2 = convertPositionToPercentage(voltageSensor2, positionMinSensor2, positionMaxSensor2);

  auto diffSensorValue = positionSensor1 - positionSensor2;
  if (diffSensorValue < 0) {
    diffSensorValue *= -1;
  }

  if (diffSensorValue > 5) {
    position = 0;

    if (errorCallback) {
      errorCallback();
    }
  } else {
    position = positionSensor1;
  }

  ESP_LOGD(TAG, "Sensor 1: voltage %d, position %d; Sensor 2: voltage %d, position %d", voltageSensor1, positionSensor1, voltageSensor2, positionSensor2);

  if (positionChangeCallback) {
    positionChangeCallback(position);
  }
}
