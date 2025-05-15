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
#include <expected>
#include <memory>
#include <vector>

#include <executor/Node.hpp>
#include <gpio/PinLevel.hpp>
#include <gpio/interface/OutputPin.hpp>

class Indicator : public executor::Node {
public:
  enum IndicatorMode : std::uint8_t {
    DISABLE_MODE = 0,
    NORMAL_MODE,
    CRUISE_READY_MODE,
    CRUISE_ON_MODE,
  };

  enum Error : std::uint8_t {
    INDICATOR_INIT_ERROR,
  };

public:
  using Pin = std::uint8_t;
  using Step = std::size_t;
  using Time = std::int64_t;
  using Pointer = std::shared_ptr<Indicator>;
  using Interval = std::uint32_t;
  using IndicatorPin = std::unique_ptr<gpio::interface::OutputPin<gpio::PinLevel>>;

public:
  struct IntervalSetting {
    Indicator::Interval interval;
    gpio::PinLevel indicatorLevel;
  };

private:
  using Intervals = std::vector<Indicator::IntervalSetting>;

public:
  static auto create(Pin pin) -> std::expected<Pointer, Error>;

private:
  explicit Indicator(IndicatorPin pin);

public:
  ~Indicator() override = default;

public:
  void setError();

public:
  void setMode(Indicator::IndicatorMode mode);

public:
  void process() override;

private:
  IndicatorPin m_indicator;

private:
  bool m_error = false;
  Step m_currentStep;
  Time m_previousTime;
  Intervals m_intervals;
  IndicatorMode m_mode;
};
