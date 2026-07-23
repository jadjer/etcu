//
// Created by jadjer on 23.07.26.
//

#pragma once

#include <driver/uart.h>
#include <algorithm>
#include "configs/configs.hpp"

namespace devices {

class Servo {
 public:
  auto init() noexcept -> void {
    // uart_config_t uart_config = {
    //     .baud_rate = 1000000, // Высокоскоростная шина ST3020
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity    = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     .source_clk = UART_SCLK_DEFAULT,
    // };
    // [[maybe_unused]] auto e1 = uart_param_config(Config::SERVO_UART, &uart_config);
    // [[maybe_unused]] auto e2 = uart_set_pin(Config::SERVO_UART, Config::SERVO_TX_PIN,
    // Config::SERVO_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // [[maybe_unused]] auto e3 = uart_driver_install(Config::SERVO_UART, 256, 256, 0, nullptr, 0);
  }

  auto set_position(std::uint16_t const target_position, SystemError& err) noexcept -> bool {
    // // Масштабирование 0-10000 во внутренний диапазон шагов ST3020 (например, 0-4095)
    // uint16_t target_steps = static_cast<uint16_t>((static_cast<uint32_t>(target_0_10000) * 4095)
    // / 10000);
    //
    // uint8_t packet[7];
    // packet[0] = 0xFF;
    // packet[1] = 0xFF;
    // packet[2] = 0x01; // ID сервопривода по умолчанию
    // packet[3] = 0x03; // Длина данных
    // packet[4] = 0x2A; // Регистр целевой позиции
    // packet[5] = static_cast<uint8_t>(target_steps >> 8);
    // packet[6] = static_cast<uint8_t>(target_steps & 0xFF);
    //
    // int written = uart_write_bytes(Config::SERVO_UART, packet, sizeof(packet));
    // return written == sizeof(packet);

    return true;
  }

  auto read_telemetry(uint16_t& out_pos, uint16_t& out_current_ma, uint8_t& out_temp_c) noexcept
      -> bool {
    // // Симуляция разбора пакета ответа UART ST3020
    // // В реальном устройстве: отправка команды READ, чтение байт с таймаутом
    // uint8_t dummy_rx[8];
    // int len = uart_read_bytes(Config::SERVO_UART, dummy_rx, sizeof(dummy_rx), 2 /
    // portTICK_PERIOD_MS);
    //
    // if (len <= 0) {
    //     // Если шина пустая в симуляции, подставляем корректные базовые данные, чтобы не падать в
    //     аварию out_pos = 0; out_current_ma = 150; out_temp_c = 42; return true;
    // }
    //
    // // Пример разбора (фиктивный для структуры пакета)
    // out_pos = 0;
    // out_current_ma = 200;
    // out_temp_c = 45;
    return true;
  }
};

}  // namespace devices
