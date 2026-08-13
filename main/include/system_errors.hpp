//
// Created by jadjer on 25.07.26.
//

#pragma once

#include <atomic>
#include "types.hpp"

class SystemErrors {
  static constexpr auto to_underlying(SystemError err) noexcept -> Error { return static_cast<Error>(err); }

  std::atomic<Error> m_errors_mask;

 public:
  constexpr SystemErrors() noexcept : m_errors_mask(0) {}

  auto add(SystemError const err) noexcept -> void { m_errors_mask.fetch_or(to_underlying(err), std::memory_order_relaxed); }
  auto update(SystemError const err) noexcept -> void { m_errors_mask.fetch_and(~to_underlying(err), std::memory_order_relaxed); }
  auto reset() noexcept -> void { m_errors_mask.store(0, std::memory_order_relaxed); }

  [[nodiscard]] auto has(SystemError const err) const noexcept -> bool {
    auto const error_mask = m_errors_mask.load(std::memory_order_relaxed);
    auto const errors = static_cast<SystemError>(error_mask);

    return has_error(errors, err);
  }

  [[nodiscard]] auto has_any() const noexcept -> bool {
    auto const error_mask = m_errors_mask.load(std::memory_order_relaxed);
    auto const errors = static_cast<SystemError>(error_mask);

    return has_error(errors);
  }

  [[nodiscard]] auto get_all() const noexcept -> SystemError {
    auto const error_mask = m_errors_mask.load(std::memory_order_relaxed);
    auto const errors = static_cast<SystemError>(error_mask);

    return errors;
  }
};
