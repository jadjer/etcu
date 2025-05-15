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
#include <expected>
#include <memory>

#include <executor/Node.hpp>
#include <gpio/InputPin.hpp>

class Switch : public executor::Node {
public:
  enum class Error : std::uint8_t {
    PIN_INIT_FAILED,
  };

public:
  using Pin = std::uint8_t;
  using Pointer = std::shared_ptr<Switch>;

public:
  static auto create(Switch::Pin pin) -> std::expected<Pointer, Error>;

private:
  explicit Switch(gpio::InputPin::Pointer pin);

public:
  ~Switch() override = default;

public:
  [[nodiscard]] virtual bool isEnabled() const;

private:
  void process() override;

private:
  gpio::InputPin::Pointer m_switch{nullptr};

private:
  bool m_enable{false};
};
