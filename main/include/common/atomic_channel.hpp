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
// Created by jadjer on 28.07.26.
//

#pragma once

#include <atomic>

namespace common {

template <class T>
class AtomicChannel {
  T m_data;
  std::atomic<bool> m_is_ready{false};

 public:
  explicit AtomicChannel() noexcept = default;
  explicit AtomicChannel(T data) noexcept : m_data{data} {}

  [[nodiscard]] auto send(T const data) -> bool {
    if (!m_is_ready.load(std::memory_order_relaxed)) {
      m_data = data;
      m_is_ready.store(true, std::memory_order_release);
      return true;
    }
    return false;
  }

  [[nodiscard]] auto receive(T& out_data) -> bool {
    if (m_is_ready.load(std::memory_order_acquire)) {
      out_data = m_data;
      m_is_ready.store(false, std::memory_order_relaxed);
      return true;
    }
    return false;
  }
};

}  // namespace commons
