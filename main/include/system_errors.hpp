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
#include "type/type.hpp"

class SystemErrors {
  static constexpr auto to_underlying(type::SystemError err) noexcept -> type::primitive::Error { return static_cast<type::primitive::Error>(err); }

  std::atomic<type::primitive::Error> m_errors_mask{0};

 public:
  auto add(type::SystemError const err) noexcept -> void { m_errors_mask.fetch_or(to_underlying(err), std::memory_order_relaxed); }
  auto update(type::SystemError const err) noexcept -> void { m_errors_mask.fetch_and(~to_underlying(err), std::memory_order_relaxed); }
  auto reset() noexcept -> void { m_errors_mask.store(0, std::memory_order_relaxed); }

  [[nodiscard]] auto has(type::SystemError const err) const noexcept -> bool {
    auto const error_mask = m_errors_mask.load(std::memory_order_relaxed);
    auto const errors = static_cast<type::SystemError>(error_mask);

    return has_error(errors, err);
  }

  [[nodiscard]] auto has_any() const noexcept -> bool {
    auto const error_mask = m_errors_mask.load(std::memory_order_relaxed);
    auto const errors = static_cast<type::SystemError>(error_mask);

    return has_error(errors);
  }

  [[nodiscard]] auto get_all() const noexcept -> type::SystemError {
    auto const error_mask = m_errors_mask.load(std::memory_order_relaxed);
    auto const errors = static_cast<type::SystemError>(error_mask);

    return errors;
  }
};
