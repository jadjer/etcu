//
// Created by jadjer on 23.07.26.
//

#pragma once

#include <atomic>
#include "concepts/concepts.hpp"
#include "core.hpp"
#include "types.hpp"

template <concepts::LoggerConcept Logger,
          concepts::AcceleratorConcept Accelerator,
          concepts::ServoConcept Servo,
          concepts::ECUConcept ECU,
          concepts::ButtonConcept ModeBtn,
          concepts::SwitchConcept Brake,
          concepts::SwitchConcept Guard,
          concepts::SwitchConcept Clutch,
          concepts::IndicatorConcept ModeInd,
          concepts::IndicatorConcept StatusInd>
class Controller {
  Logger& m_logger;
  Accelerator& m_accelerator;
  Servo& m_servo;
  ECU& m_ecu;
  ModeBtn& m_mode_button;
  Brake& m_brake;
  Guard& m_guard;
  Clutch& m_clutch;
  ModeInd& m_mode_indicator;
  StatusInd& m_status_indicator;

  Core m_core;

  // Атомарные мосты межъядерного взаимодействия
  std::atomic<Mode> m_current_mode{Mode::Normal};
  std::atomic<SystemError> m_system_errors{SystemError::None};
  std::atomic<DriverInput> m_driver_input{DriverInput{}};
  std::atomic<uint16_t> m_target_cruise_pos{0};
  std::atomic<uint16_t> m_actual_servo_pos{0};

 public:
  Controller(Logger& logger,
             Accelerator& accelerator,
             Servo& servo,
             ECU& ecu,
             ModeBtn& mode_button,
             Brake& brake,
             Guard& guard,
             Clutch& clutch,
             ModeInd& mode_indicator,
             StatusInd& status_indicator) noexcept
      : m_logger(logger),
        m_accelerator(accelerator),
        m_servo(servo),
        m_ecu(ecu),
        m_mode_button(mode_button),
        m_brake(brake),
        m_guard(guard),
        m_clutch(clutch),
        m_mode_indicator(mode_indicator),
        m_status_indicator(status_indicator) {}

  auto init() noexcept -> void {
    m_logger.init();
    m_logger.log_info("Starting Throttle Controller Initialization...");

    // Инициализация периферии
    m_servo.init();
    m_accelerator.init();
    m_ecu.init();
    m_mode_button.init();
    m_clutch.init();
    m_brake.init();
    m_guard.init();
    m_mode_indicator.init();
    m_status_indicator.init();

    // m_core.init(m_adc_handle);

    m_logger.log_info("Executing automatic zero and range calibration...");
    // m_core.execute_auto_calibration();
  }

  // Core 0 Task: Системная бизнес-логика, Конечный автомат (FSM), Отказоустойчивость
  auto process_system_loop() noexcept -> void {
    // m_mode_button.update();

    // Считывание текущего слепка атомиков из критического ядра
    SystemError current_errs = m_system_errors.load(std::memory_order_relaxed);
    Mode current_mode = m_current_mode.load(std::memory_order_relaxed);
    DriverInput input = m_driver_input.load(std::memory_order_relaxed);

    // Обработка критической ошибки аппаратного сторожа Guard (Мгновенное отключение)
    // if (m_guard.is_active()) {
    //   m_current_mode.store(Mode::Emergency, std::memory_order_release);
    //   m_system_errors.store(current_errs | SystemError::AcceleratorMismatch,
    //   std::memory_order_release);
    // }

    // Конечный автомат переключения режимов
    // switch (current_mode) {
    // case Mode::Normal:
    //   if (current_errs != SystemError::None) {
    //     m_current_mode.store(Mode::Emergency, std::memory_order_release);
    //   } else if (m_mode_button.is_long_press() && m_actual_servo_pos.load() == 0) {
    //     // Калибровка нуля по долгому нажатию на нулевой скорости
    //     // m_core.force_set_zero_calibration(input.raw_hall_1, input.raw_hall_2);
    //     Logger::log_info("Manual Zero Calibrated via Long Press.");
    //   } else if (m_mode_button.is_long_press() && m_actual_servo_pos.load() > 0) {
    //     // Переход в Круиз Контроль
    //     m_target_cruise_pos.store(m_actual_servo_pos.load(), std::memory_order_relaxed);
    //     // m_current_mode.store(Mode::CruiseControl, std::memory_order_release);
    //     Logger::log_info("Switched to Cruise Control Mode.");
    //   }
    //   break;
    //
    // case Mode::Eco:
    //   if (current_errs != SystemError::None) {
    //     m_current_mode.store(Mode::Emergency, std::memory_order_release);
    //   } else if (m_mode_button.is_short_press() || m_clutch.is_active() || m_brake.is_active()) {
    //     // Сброс круиза в обычный режим
    //     m_current_mode.store(Mode::Normal, std::memory_order_release);
    //     Logger::log_info("Cruise Control Deactivated.");
    //   }
    //   break;
    //
    // case Mode::Emergency:
    //   if (m_mode_button.is_short_press() && !m_guard.is_active()) {
    //     // Попытка сброса аварии коротким нажатием с очисткой калибровки
    //     // m_core.reset_calibration();
    //     m_system_errors.store(SystemError::None, std::memory_order_release);
    //     m_current_mode.store(Mode::Normal, std::memory_order_release);
    //     Logger::log_info("Emergency recovered. Hard reset calibration applied.");
    //   }
    //   break;
    // }

    // Обновление индикации и передача пакетов в ECU
    // m_mode_indicator.set_status(current_mode, current_errs);
    // m_ecu.update(current_mode, m_actual_servo_pos.load(), current_errs);
  }

  // Core 1 Task: Hard Real-Time Секция (Сверхбыстрый расчет без задержек ввода-вывода)
  auto process_critical_loop() noexcept -> void {
    SystemError local_errors = m_system_errors.load(std::memory_order_relaxed);
    Mode local_mode = m_current_mode.load(std::memory_order_relaxed);

    // Чтение датчиков Холла
    int raw1 = 0, raw2 = 0;
    // adc_oneshot_read(m_adc_handle, config::ADC_HALL_1, &raw1);
    // adc_oneshot_read(m_adc_handle, config::ADC_HALL_2, &raw2);

    // Публикация входных данных для Core 0
    // DriverInput current_input{.raw_hall_1 = static_cast<uint16_t>(raw1),
    //                           .raw_hall_2 = static_cast<uint16_t>(raw2),
    //                           .clutch_pressed = m_clutch.is_active(),
    //                           .brake_pressed = m_brake.is_active(),
    //                           .guard_active = m_guard.is_active()};
    // m_driver_input.store(current_input, std::memory_order_relaxed);

    // Чтение фоновой телеметрии сервопривода ST3020 (ошибки тока/температуры)
    // m_servo.read_telemetry(local_errors);

    // Обработка данных вычислительным ядром
    // uint16_t computed_pos = m_core.process(current_input, local_mode, m_target_cruise_pos.load(),
    // local_errors);
    //
    // m_actual_servo_pos.store(computed_pos, std::memory_order_relaxed);
    m_system_errors.store(local_errors, std::memory_order_release);
  }
};
