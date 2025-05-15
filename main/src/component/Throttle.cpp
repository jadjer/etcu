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

#include "component/Throttle.hpp"

#include <utility>

#include <esp_log.h>

auto const TAG = "Throttle";
auto const POSITION_MINIMAL = 0;
auto const POSITION_MAXIMAL = 100;

void Throttle::registerErrorCallback(Throttle::ErrorCallback callback) { m_errorCallback = std::move(callback); }

void Throttle::registerPositionChangeCallback(Throttle::PositionChangeCallback callback) { m_positionChangeCallback = std::move(callback); }

void Throttle::setPosition(Throttle::Position position) const {
  auto requiredPosition = position;

  if (requiredPosition < POSITION_MINIMAL) {
    requiredPosition = POSITION_MINIMAL;
    ESP_LOGW(TAG, "Limiting the throttle position to the minimum value of %d", POSITION_MINIMAL);
  }

  if (requiredPosition > POSITION_MAXIMAL) {
    requiredPosition = POSITION_MAXIMAL;
    ESP_LOGW(TAG, "Limiting the throttle position to the maximum value of %d", POSITION_MAXIMAL);
  }

  //  m_throttle.setPercent(setPercentage);

  ESP_LOGI(TAG, "Throttle position is set to %d", requiredPosition);
}

void Throttle::process() {}
