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
#include <gpio/InputPin.hpp>

class ModeButton : public executor::Node {
public:
  enum class Error : std::uint8_t {
    BUTTON_INIT_FAILED,
  };

public:
  using Pin = std::uint8_t;
  using Time = std::int64_t;
  using Pointer = std::shared_ptr<ModeButton>;
  using HoldCallback = std::function<void()>;
  using PressCallback = std::function<void()>;

public:
  static auto create(ModeButton::Pin pin, ModeButton::Time holdTimeInUS = 1000000, ModeButton::Time thresholdInUS = 100000) -> std::expected<Pointer, Error>;

private:
  ModeButton(gpio::InputPin::Pointer button, ModeButton::Time holdTimeInUS, ModeButton::Time thresholdInUS);

public:
  ~ModeButton() override = default;

private:
  void process() override;
  void processButtonPressed();
  void processButtonReleased();

private:
  gpio::InputPin::Pointer const button;
  ModeButton::Time const m_holdTime;
  ModeButton::Time const m_threshold;

private:
  bool m_isHeld;
  bool m_isPressed;

private:
  ModeButton::Time m_pressTime;
  ModeButton::Time m_releaseTime;
};
