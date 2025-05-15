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

#include "component/ModeButton.hpp"

#include <utility>

#include <esp_timer.h>

auto ModeButton::create(ModeButton::Pin const pin, ModeButton::Time holdTimeInUS, ModeButton::Time const thresholdInUS) -> std::expected<ModeButton::Pointer, ModeButton::Error> {
  auto button_ = gpio::InputPin::create(pin);
  if (not button_) {
    return std::unexpected(ModeButton::Error::BUTTON_INIT_FAILED);
  }

  return ModeButton::Pointer(new ModeButton(std::move(*button_), holdTimeInUS, thresholdInUS));
}

ModeButton::ModeButton(gpio::InputPin::Pointer button, ModeButton::Time const holdTimeInUS, ModeButton::Time const thresholdInUS)
    : button(std::move(button)),

      m_holdTime(holdTimeInUS), m_threshold(thresholdInUS),

      m_isHeld(false), m_isPressed(false),

      m_pressTime(0), m_releaseTime(0) {}

void ModeButton::process() {
  auto const buttonState = button->getLevel();

  if (buttonState == gpio::PinLevel::HIGH) {
    return processButtonReleased();
  }

  if (buttonState == gpio::PinLevel::LOW) {
    return processButtonPressed();
  }
}

void ModeButton::processButtonPressed() {
  if (m_isHeld) {
    return;
  }

  auto const currentTime = esp_timer_get_time();
  auto const idleTime = currentTime - m_releaseTime;
  if (idleTime < m_threshold) {
    return;
  }

  if (not m_isPressed) {
    m_isPressed = true;
    m_pressTime = currentTime;

    return;
  }

  auto const holdTime = currentTime - m_pressTime;
  if (holdTime < m_holdTime) {
    return;
  }

  m_isHeld = true;

  //  if (m_holdCallback) {
  //    m_holdCallback();
  //  }
}

void ModeButton::processButtonReleased() {
  if (not m_isPressed) {
    return;
  }

  if (not m_isHeld) {
    //    if (m_pressCallback) {
    //      m_pressCallback();
    //    }
  }

  m_releaseTime = esp_timer_get_time();
  m_isHeld = false;
  m_isPressed = false;
}
