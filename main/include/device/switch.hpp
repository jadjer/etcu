//
// Created by jadjer on 18.08.26.
//

#pragma once

#include "concepts.hpp"

namespace device {

template <class Driver>
  requires concepts::GPIO<Driver>

class Switch {
  Driver& m_driver;

 public:
  explicit Switch(Driver& driver) : m_driver{driver} {}

  [[nodiscard]] auto init() noexcept -> type::SystemError {
    if (!m_driver.init())
      return type::SystemError::ButtonInitFault;

    return type::SystemError::None;
  }

  [[nodiscard]] auto is_active() noexcept -> bool { return m_driver.get_level(); }
};

}  // namespace device
