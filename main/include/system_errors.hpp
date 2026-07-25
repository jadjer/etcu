//
// Created by jadjer on 25.07.26.
//

#pragma once

#include "types.hpp"

class SystemErrors {
  static constexpr auto to_underlying(SystemError err) noexcept -> Error { return static_cast<Error>(err); }

  std::atomic<Error> m_errors_mask;

 public:
  constexpr SystemErrors() noexcept : m_errors_mask(0) {}

  auto add(SystemError const err) noexcept -> void { m_errors_mask.fetch_or(to_underlying(err), std::memory_order_relaxed); }
  auto update(SystemError const err) noexcept -> void { m_errors_mask.fetch_and(~to_underlying(err), std::memory_order_relaxed); }
  auto reset() noexcept -> void { m_errors_mask.store(0, std::memory_order_relaxed); }

  [[nodiscard]] auto has(SystemError const err) const noexcept -> bool { return (m_errors_mask.load(std::memory_order_relaxed) & to_underlying(err)) != 0; }
  [[nodiscard]] auto has_any() const noexcept -> bool { return m_errors_mask.load(std::memory_order_relaxed) != 0; }
  [[nodiscard]] auto get_all() const noexcept -> SystemError { return static_cast<SystemError>(m_errors_mask.load(std::memory_order_relaxed)); }
};
