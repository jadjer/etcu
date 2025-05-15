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

#include "component/Switch.hpp"

#include <utility>

auto Switch::create(Switch::Pin pin) -> std::expected<Switch::Pointer, Switch::Error> {
  auto switchInstance = gpio::InputPin::create(pin, gpio::PinLevel::LOW);
  if (not switchInstance) {
    return std::unexpected(Switch::Error::PIN_INIT_FAILED);
  }

  return Switch::Pointer(new Switch(std::move(*switchInstance)));
}

Switch::Switch(gpio::InputPin::Pointer pin) : m_switch(std::move(pin)) {}

bool Switch::isEnabled() const { return m_enable; }

void Switch::process() {
  auto const level = m_switch->getLevel();

  if (level == gpio::PinLevel::LOW) {

    if (m_enable) {
      m_enable = false;

//      if (m_switchCallback) {
//        m_switchCallback(m_enable);
//      }
    }
  }

  if (level == gpio::PinLevel::HIGH) {

    if (not m_enable) {
      m_enable = true;

//      if (m_switchCallback) {
//        m_switchCallback(m_enable);
//      }
    }
  }
}
