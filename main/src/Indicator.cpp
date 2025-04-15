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

#include "Indicator.hpp"

#include <esp_timer.h>

auto const MICROSECONDS_IN_SECOND = 1000000;
auto const MICROSECONDS_IN_MILLISECOND = 1000;

Indicator::Indicator(Pin pin) : m_indicator(pin, gpio::PIN_LEVEL_LOW), m_currentStep(0), m_previousTime(0), m_mode(DISABLE_MODE), m_intervals() {}

void Indicator::setError() {
  m_error = true;
  m_intervals = {
      {100 * MICROSECONDS_IN_MILLISECOND, gpio::PIN_LEVEL_HIGH},
      {100 * MICROSECONDS_IN_MILLISECOND, gpio::PIN_LEVEL_LOW},
  };
}

void Indicator::setMode(Indicator::IndicatorMode mode) {
  if (m_error) {
    return;
  }

  if (mode == m_mode) {
    return;
  }

  switch (mode) {
  case DISABLE_MODE:
  case NORMAL_MODE:
    m_intervals.clear();
    break;
  case CRUISE_READY_MODE:
    m_intervals = {
        {1000 * MICROSECONDS_IN_MILLISECOND, gpio::PIN_LEVEL_HIGH},
        {100 * MICROSECONDS_IN_MILLISECOND, gpio::PIN_LEVEL_LOW},
    };
    break;
  case CRUISE_ON_MODE:
    m_intervals = {
        {1 * MICROSECONDS_IN_SECOND, gpio::PIN_LEVEL_HIGH},
    };
    break;
  }

  m_mode = mode;
}

void Indicator::process() {
  if (m_intervals.empty()) {
    m_indicator.setLevel(gpio::PIN_LEVEL_LOW);
    return;
  }

  if (m_currentStep >= m_intervals.size()) {
    m_currentStep = 0;
  }

  auto const currentTime = esp_timer_get_time();
  auto const timeDifferent = currentTime - m_previousTime;

  auto const currentIntervalSetting = m_intervals.at(m_currentStep);
  auto const currentInterval = currentIntervalSetting.interval;

  if (timeDifferent < currentInterval) {
    return;
  }

  auto const currentIndicatorLevel = currentIntervalSetting.indicatorLevel;

  m_indicator.setLevel(currentIndicatorLevel);
  m_currentStep++;
  m_previousTime = currentTime;
}
