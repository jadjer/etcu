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

template <class T>
class AtomicValue {
  std::atomic<T> m_data;

 public:
  constexpr explicit AtomicValue(T data) noexcept : m_data{data} {}

  constexpr auto set(T const data) noexcept -> void { m_data.store(data, std::memory_order_relaxed); }

  [[nodiscard]] constexpr auto get() noexcept -> T { return m_data.load(std::memory_order_relaxed); }
};

}  // namespace commons
