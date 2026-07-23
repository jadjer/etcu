//
// Created by jadjer on 23.07.26.
//

#pragma once

#include <algorithm>
#include <cmath>
#include "configs/configs.hpp"
#include "types.hpp"

class Core {
 public:
  [[nodiscard]] auto calculate_servo_position(Mode mode,
                                              float acc_percent,
                                              bool cruise_active,
                                              float current_speed,
                                              float target_speed) noexcept -> float {
    if (mode == Mode::Emergency) {
      return 0.0f;  // Безопасное состояние дросселя
    }

    float base_target = 0.0f;

    // Расчет базового положения от педали в зависимости от режима
    if (mode == Mode::Normal) {
      base_target = acc_percent;  // Линейно
    } else if (mode == Mode::Eco) {
      // Нелинейная кривая экспоненты (плавный старт)
      base_target = std::pow(acc_percent / 100.0f, 1.5f) * 100.0f;
    }

    // Коррекция ПИ-регулятором, если активен Круиз-Контроль
    if (cruise_active) {
      float const speed_error = target_speed - current_speed;
      // Простейший П-регулятор для удержания
      float const cruise_modifier = speed_error * 2.5f;
      base_target = std::clamp(base_target + cruise_modifier, 0.0f, 100.0f);
    }

    return std::clamp(base_target, 0.0f, 100.0f);
  }
};
