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
// Created by jadjer on 25.07.26.
//

#pragma once

#include <atomic>

#include "type/error.hpp"

class SystemErrors {
  std::atomic<type::SystemError> m_errors_mask{type::SystemError::None};

 public:
  constexpr SystemErrors() noexcept = default;

  SystemErrors(SystemErrors const&) noexcept = delete;
  auto operator=(SystemErrors const&) noexcept -> SystemErrors& = delete;

  SystemErrors(SystemErrors&&) noexcept = delete;
  auto operator=(SystemErrors&&) noexcept -> SystemErrors& = delete;

  constexpr ~SystemErrors() noexcept = default;

  auto update(type::SystemError const device_mask, type::SystemError const active_errors) noexcept -> void {
    type::SystemError current = m_errors_mask.load(std::memory_order_relaxed);

    while (true) {
      if (type::SystemError const next = (current & ~device_mask) | (active_errors & device_mask);
          m_errors_mask.compare_exchange_weak(current, next, std::memory_order_release, std::memory_order_relaxed)) {
        break;
      }
    }
  }

  auto reset() noexcept -> void { m_errors_mask.store(type::SystemError::None, std::memory_order_relaxed); }

  [[nodiscard]] auto has(type::SystemError const err_or_group) const noexcept -> bool {
    return type::has_error(m_errors_mask.load(std::memory_order_relaxed), err_or_group);
  }

  [[nodiscard]] auto has_any() const noexcept -> bool { return type::has_error(m_errors_mask.load(std::memory_order_relaxed)); }

  [[nodiscard]] auto get_error_mask() const noexcept -> type::SystemError { return m_errors_mask.load(std::memory_order_relaxed); }
};
