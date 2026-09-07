//
// Created by jadjer on 7.09.26.
//

#pragma once

enum class CruiseState : std::uint8_t { Idle, CheckingStability, Active };

class CruiseManager {
  static constexpr float loop_dt{0.01f};

  CruiseState m_state{CruiseState::Idle};
  type::Speed m_base_speed{0};
  std::uint32_t m_stability_ticks{0};

  common::AtomicContainer<type::Speed> m_target_speed{0};

 public:
  auto set_target_speed(type::Speed const speed) noexcept -> void {
    m_target_speed.store(speed);
    m_state = CruiseState::Active;
    ESP_LOGI("CRUISE", "Target speed overridden by button to: %d", speed);
  }

  auto get_target_speed(type::Speed const current_speed, type::Control const& control, bool const safety_active) noexcept -> type::Speed {
    if (safety_active) {
      if (m_target_speed > 0 || m_state != CruiseState::Idle) {
        m_target_speed = type::Speed{0};
        m_state = CruiseState::Idle;
      }
      return type::Speed{0};
    }

    switch (m_state) {
      case CruiseState::Idle:
        // Проверка нижней границы полностью удалена.
        // Отслеживание стабильности начинается на любой скорости, если включен тумблер.
        if (control.cruise.enabled && m_target_speed == 0) {
          m_base_speed = current_speed;
          m_stability_ticks = 0;
          m_state = CruiseState::CheckingStability;
        }
        break;

      case CruiseState::CheckingStability:
        if (!control.cruise.enabled) {
          m_target_speed = type::Speed{0};
          m_state = CruiseState::Idle;
          is_target_changed = true;
          break;
        }

        // Проверяем стабильность скорости в коридоре допуска
        if (std::abs(current_speed.get() - m_base_speed.get()) <= control.cruise.tolerance.get()) {
          m_stability_ticks++;

          float const delay_seconds = control.cruise.delay;
          if (m_stability_ticks >= static_cast<std::uint32_t>(delay_seconds / loop_dt)) {
            // Скорость стабилизировалась -> ГЕНЕРИРУЕМ ТАРГЕТ
            m_target_speed = current_speed;
            m_state = CruiseState::Active;
            is_target_changed = true;
          }
        } else {
          // Скорость изменилась: плавно смещаем базовую точку окна стабильности
          m_base_speed = current_speed;
          m_stability_ticks = 0;
          // Проверка падения ниже speed_start_fade удалена. Окно просто смещается.
        }
        break;

      case CruiseState::Active:
        // Условие выхода по минимальной скорости удалено.
        // Выключаемся только если водитель выключил тумблер круиза.
        if (!control.cruise.enabled) {
          m_target_speed = type::Speed{0};
          m_state = CruiseState::Idle;
          is_target_changed = true;
        }
        break;
    }

    return is_target_changed;
  }
};
