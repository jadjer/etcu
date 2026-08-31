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
    if (!m_driver.init()) [[unlikely]]
      return type::SystemError::ButtonInitFault;

    return type::SystemError::None;
  }

  [[nodiscard]] auto is_active() noexcept -> bool { return m_driver.get_level(); }
};

}  // namespace device
