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
// Created by jadjer on 28.08.26.
//

#pragma once

#include <atomic>
#include <concepts>

namespace common {

template <typename T>
  requires std::is_trivially_copyable_v<T> && std::default_initializable<T>
class AtomicContainer {
  T m_buffer_a{};
  T m_buffer_b{};

  std::atomic<T*> m_active_ptr{&m_buffer_a};

 public:
  constexpr explicit AtomicContainer(T const& initial_data) noexcept {
    m_buffer_a = initial_data;
    m_buffer_b = initial_data;
    m_active_ptr.store(&m_buffer_a, std::memory_order_release);
  }

  constexpr AtomicContainer() noexcept = default;

  AtomicContainer(AtomicContainer const&) = delete;
  auto operator=(AtomicContainer const&) -> AtomicContainer& = delete;

  AtomicContainer(AtomicContainer&&) = delete;
  auto operator=(AtomicContainer&&) -> AtomicContainer& = delete;

  ~AtomicContainer() = default;

  [[nodiscard]] auto load() const noexcept -> T {
    T* const active = m_active_ptr.load(std::memory_order_acquire);

    return *active;
  }

  auto store(T const& data) noexcept -> void {
    T* const active = m_active_ptr.load(std::memory_order_acquire);
    T* const shadow = (active == &m_buffer_a) ? &m_buffer_b : &m_buffer_a;

    *shadow = data;

    m_active_ptr.store(shadow, std::memory_order_release);
  }
};

}  // namespace common
