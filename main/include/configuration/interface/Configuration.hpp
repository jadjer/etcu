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

namespace interface {

using Pin = std::uint8_t;

class Configuration {
public:
  virtual ~Configuration() = default;

public:
  [[maybe_unused]] [[nodiscard]] virtual Pin getIndicatorPin() const = 0;
  [[maybe_unused]] [[nodiscard]] virtual Pin getModeButtonPin() const = 0;
  [[maybe_unused]] [[nodiscard]] virtual Pin getBreakSwitchPin() const = 0;
  [[maybe_unused]] [[nodiscard]] virtual Pin getGuardSwitchPin() const = 0;
  [[maybe_unused]] [[nodiscard]] virtual Pin getClutchSwitchPin() const = 0;
  [[maybe_unused]] [[nodiscard]] virtual Pin getTwistSensor1Pin() const = 0;
  [[maybe_unused]] [[nodiscard]] virtual Pin getTwistSensor2Pin() const = 0;
};

}// namespace interface

#include <memory>

using ConfigurationPtr = std::shared_ptr<interface::Configuration>;
