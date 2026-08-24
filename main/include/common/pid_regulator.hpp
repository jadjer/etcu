//
// Created by jadjer on 21.08.26.
//

#pragma once

#include <algorithm>

namespace common {

struct PidCoefficients {
  float kp{0.0f};
  float ki{0.0f};
  float kd{0.0f};
};

template <float MinLimit, float MaxLimit>
class PidRegulator {
  static constexpr float min_limit{MinLimit};
  static constexpr float max_limit{MaxLimit};

  float m_dt{0.0f};
  float m_integral{0.0f};
  float m_last_error{0.0f};
  PidCoefficients m_coefficients;

 public:
  constexpr explicit PidRegulator(PidCoefficients const& coefficients, float const dt_sec) noexcept : m_dt(dt_sec), m_coefficients(coefficients) {}

  template <typename T>
    requires std::is_convertible_v<T, float>
  [[nodiscard]] auto calculate(T const target, T const current) const noexcept -> float {
    auto const error = static_cast<float>(target) - static_cast<float>(current);

    float const p_term = m_coefficients.kp * error;
    float const i_term = m_coefficients.ki * m_integral;
    float const d_term = m_coefficients.kd * ((error - m_last_error) / m_dt);

    return p_term + i_term + d_term;
  }

  template <typename T>
    requires std::is_convertible_v<T, float>
  auto update(T const target, T const current, bool const freeze_integral = false) noexcept -> void {
    auto const error = static_cast<float>(target) - static_cast<float>(current);

    if (!freeze_integral) {
      m_integral += static_cast<float>(error) * m_dt;
      m_integral = std::clamp(m_integral, min_limit, max_limit);
    }

    m_last_error = error;
  }

  auto reset() noexcept {
    m_integral = 0.0f;
    m_last_error = 0.0f;
  }
};

}  // namespace common
