// Copyright 2026 Pavel Suprunov
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

//
// Created by jadjer on 18.08.26.
//

#pragma once

#include "config/concepts.hpp"

namespace device {

template <class Driver>
  requires concepts::GPIO<Driver>
class Switch {
  Driver& m_driver;

 public:
  constexpr explicit Switch(Driver& driver) : m_driver{driver} {}

  constexpr Switch() noexcept = delete;

  Switch(Switch const&) noexcept = delete;
  auto operator=(Switch const&) noexcept -> Switch& = delete;

  Switch(Switch&&) noexcept = delete;
  auto operator=(Switch&&) noexcept -> Switch& = delete;

  constexpr ~Switch() noexcept = default;

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_driver.init()) [[unlikely]] {
      return type::SystemError::ButtonInitFault;
    }

    return type::SystemError::None;
  }

  [[nodiscard]] auto is_active() noexcept -> bool { return m_driver.get_level(); }
};

}  // namespace device
