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
#include <functional>

#include <executor/Node.hpp>

class Throttle : public executor::Node {
public:
  using Pointer = std::shared_ptr<Throttle>;
  using Position = std::uint8_t;
  using ErrorCallback = std::function<void()>;
  using PositionChangeCallback = std::function<void(Throttle::Position)>;

public:
  ~Throttle() override = default;

public:
  void registerErrorCallback(Throttle::ErrorCallback callback);
  void registerPositionChangeCallback(Throttle::PositionChangeCallback callback);

public:
  void setPosition(Throttle::Position position) const;

private:
  void process() override;

private:
  Throttle::ErrorCallback m_errorCallback = nullptr;
  Throttle::PositionChangeCallback m_positionChangeCallback = nullptr;
};
