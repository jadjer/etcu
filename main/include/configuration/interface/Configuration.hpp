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

namespace interface {

class Configuration {
public:
  using Pin = std::uint8_t;

public:
  virtual ~Configuration() = default;

public:
  [[nodiscard]] [[maybe_unused]] virtual auto getIndicatorPin() const -> Pin = 0;
  [[nodiscard]] [[maybe_unused]] virtual auto getModeButtonPin() const -> Pin = 0;
  [[nodiscard]] [[maybe_unused]] virtual auto getBreakSwitchPin() const -> Pin = 0;
  [[nodiscard]] [[maybe_unused]] virtual auto getGuardSwitchPin() const -> Pin = 0;
  [[nodiscard]] [[maybe_unused]] virtual auto getClutchSwitchPin() const -> Pin = 0;
  [[nodiscard]] [[maybe_unused]] virtual auto getTwistSensor1Pin() const -> Pin = 0;
  [[nodiscard]] [[maybe_unused]] virtual auto getTwistSensor2Pin() const -> Pin = 0;
};

} // namespace interface

using ConfigurationSharedPtr = std::shared_ptr<interface::Configuration>;
using ConfigurationUniquePtr = std::unique_ptr<interface::Configuration>;
using ConfigurationWeakPtr = std::weak_ptr<interface::Configuration>;
