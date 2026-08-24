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
// Created by jadjer on 3.08.26.
//

#pragma once

#include <atomic>

namespace common {

template <typename T>
class AtomicValue {
  T m_data{};
  mutable std::atomic_flag m_lock = ATOMIC_FLAG_INIT;

 public:
  constexpr AtomicValue() noexcept = default;
  constexpr explicit AtomicValue(T const& data) noexcept : m_data{data} {}

  AtomicValue(AtomicValue const&) = delete;
  auto operator=(AtomicValue const&) -> AtomicValue& = delete;

  auto set(T const& data) noexcept -> void {
    while (m_lock.test_and_set(std::memory_order_acquire)) {
    }

    m_data = data;

    m_lock.clear(std::memory_order_release);
  }

  [[nodiscard]] auto get() const noexcept -> T {
    while (m_lock.test_and_set(std::memory_order_acquire)) {
    }

    T const current_state = m_data;

    m_lock.clear(std::memory_order_release);

    return current_state;
  }
};

}  // namespace common
