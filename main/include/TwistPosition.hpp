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

#include <adc/Channel.hpp>
#include <adc/Unit.hpp>
#include <concepts>
#include <cstdint>
#include <executor/Node.hpp>
#include <functional>

class TwistPosition : public executor::Node {
public:
  using Position = std::uint8_t;
  using ErrorCallback = std::function<void()>;
  using PositionChangeCallback = std::function<void(Position)>;

private:
  using SensorPosition = std::uint16_t;

private:
  using UnitPointer = std::unique_ptr<adc::Unit>;
  using ChannelPointer = std::unique_ptr<adc::Channel>;

public:
  TwistPosition();
  ~TwistPosition() override = default;

public:
  auto registerErrorCallback(ErrorCallback callback) -> void;
  auto registerPositionChangeCallback(PositionChangeCallback callback) -> void;

public:
  [[nodiscard]] auto getPosition() const -> Position;

private:
  auto process() -> void override;

private:
  ErrorCallback errorCallback = nullptr;
  PositionChangeCallback positionChangeCallback = nullptr;

private:
  UnitPointer unit = nullptr;
  ChannelPointer sensor1 = nullptr;
  ChannelPointer sensor2 = nullptr;

private:
  Position position = 0;
  SensorPosition positionMinSensor1 = 0;
  SensorPosition positionMaxSensor1 = 0;
  SensorPosition positionMinSensor2 = 0;
  SensorPosition positionMaxSensor2 = 0;
};
