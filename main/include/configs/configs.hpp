//
// Created by jadjer on 23.07.26.
//

#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "hal/adc_types.h"

namespace configs {

struct Pins {
  struct UART {
    struct Logger {
      static constexpr gpio_num_t Tx = GPIO_NUM_34;
      static constexpr gpio_num_t Rx = GPIO_NUM_35;
    };

    struct Servo {
      static constexpr gpio_num_t Tx = GPIO_NUM_17;
      static constexpr gpio_num_t Rx = GPIO_NUM_18;
    };

    struct Ecu {
      static constexpr gpio_num_t Tx = GPIO_NUM_19;
      static constexpr gpio_num_t Rx = GPIO_NUM_20;
    };
  };

  static constexpr gpio_num_t Button = GPIO_NUM_1;
  static constexpr gpio_num_t Clutch = GPIO_NUM_2;
  static constexpr gpio_num_t Brake = GPIO_NUM_3;
  static constexpr gpio_num_t Guard = GPIO_NUM_4;
  static constexpr gpio_num_t Led = GPIO_NUM_5;
};

struct UART {
  static constexpr uart_port_t Log = UART_NUM_0;
  static constexpr uart_port_t Servo = UART_NUM_1;
  static constexpr uart_port_t Ecu = UART_NUM_2;
};

struct ADC {
  static constexpr adc_channel_t Hall1 = ADC_CHANNEL_0;
  static constexpr adc_channel_t Hall2 = ADC_CHANNEL_1;
};

struct Safety {
  static constexpr std::uint16_t HallMismatchThreshold = 150;
  static constexpr std::uint32_t LongPressMs = 1500;
};

struct System {
  static constexpr std::uint8_t SystemCore = 0;
  static constexpr std::uint8_t CriticalCore = 1;
  static constexpr std::uint16_t SystemRate = 50;
  static constexpr std::uint16_t CriticalRate = 50;
};

}  // namespace configs
