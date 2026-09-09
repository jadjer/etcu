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
// Created by jadjer on 21.08.26.
//

#pragma once

#include <algorithm>

namespace common {

class PidController {
  float const m_kp, m_ki, m_kd, m_dt;
  float const m_min, m_max;
  float m_integral{0.0f};
  float m_last_error{0.0f};

public:
  constexpr PidController(float kp, float ki, float kd, float dt, float min, float max) noexcept
      : m_kp(kp), m_ki(ki), m_kd(kd), m_dt(dt), m_min(min), m_max(max) {}

  [[nodiscard]] auto calculate(float const error) const noexcept -> float {
    float const p_term = m_kp * error;
    float const i_term = m_integral;
    float const d_term = m_kd * ((error - m_last_error) / m_dt);

    return std::clamp(p_term + i_term + d_term, m_min, m_max);
  }

  auto update(float const error, bool const freeze_integral) noexcept -> void {
    if (!freeze_integral) {
      m_integral = std::clamp(m_integral + m_ki * error * m_dt, m_min, m_max);
    }

    m_last_error = error;
  }

  auto reset_to(float const base_value, float const initial_error = 0.0f) noexcept -> void {
    m_integral = std::clamp(base_value, m_min, m_max);
    m_last_error = initial_error;
  }
};

}  // namespace common
