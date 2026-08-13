//
// Created by jadjer on 3.08.26.
//

#pragma once

#include <atomic>

namespace commons {

template <typename T>
class AtomicValue {
  std::atomic<T> m_data;

 public:
  explicit AtomicValue(T data) : m_data{data} {}

  auto set(T const data) -> void { m_data.store(data, std::memory_order_relaxed); }

  auto get() -> T { return m_data.load(std::memory_order_relaxed); }
};

}  // namespace commons
