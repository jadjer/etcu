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

#include "config.hpp"
#include "controller.hpp"
#include "system_host.hpp"

namespace {

config::driver::ADC adc_driver;
config::device::Accelerator accelerator{adc_driver};

config::driver::UARTServo uart_servo_driver;
config::driver::PowerEnable power_enable_driver;
config::device::Servo servo{uart_servo_driver, power_enable_driver};

config::driver::UARTEcu uart_ecu_driver;
config::device::ECU ecu{uart_ecu_driver};

config::driver::ButtonMode button_mode_driver;
config::device::ButtonMode mode_button{button_mode_driver};

config::driver::LedMode led_mode_driver;
config::device::LedMode mode_indicator{led_mode_driver};

config::driver::SwitchBrake switch_brake_driver;
config::device::Brake brake_switch{switch_brake_driver};

config::driver::SwitchGuard switch_guard_driver;
config::device::Guard guard_switch{switch_guard_driver};

Controller controller{accelerator, servo, ecu, mode_button, mode_indicator, brake_switch, guard_switch};

SystemHost system_host{controller};

}  // namespace

extern "C" void app_main() {
  controller.init();

  system_host.run();
}
