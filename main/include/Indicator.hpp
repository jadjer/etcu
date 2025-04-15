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

#include <cstddef>
#include <cstdint>
#include <executor/Node.hpp>
#include <gpio/OutputPin.hpp>
#include <vector>

class Indicator : public executor::Node {
public:
  using Pin = std::uint8_t;
  using Step = std::size_t;
  using Time = std::int64_t;
  using Interval = std::uint32_t;

  enum IndicatorMode : std::uint8_t {
    DISABLE_MODE = 0,
    NORMAL_MODE,
    CRUISE_READY_MODE,
    CRUISE_ON_MODE,
  };

  struct IntervalSetting {
    Indicator::Interval interval;
    gpio::PinLevel indicatorLevel;
  };

  using Intervals = std::vector<Indicator::IntervalSetting>;

public:
  explicit Indicator(Pin pin);
  ~Indicator() override = default;

public:
  void setError();

public:
  void setMode(Indicator::IndicatorMode mode);

public:
  void process() override;

private:
  gpio::OutputPin m_indicator;

private:
  bool m_error = false;
  Indicator::Step m_currentStep;
  Indicator::Time m_previousTime;
  Indicator::IndicatorMode m_mode;
  Indicator::Intervals m_intervals;
};
