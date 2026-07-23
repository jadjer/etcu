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

#include "controller.hpp"
#include "devices/accelerator.hpp"
#include "devices/button.hpp"
#include "devices/ecu.hpp"
#include "devices/indicator.hpp"
#include "devices/servo.hpp"
#include "devices/switch.hpp"
#include "logger.hpp"
#include "system_host.hpp"

namespace {

Logger logger;

devices::Accelerator accelerator;
devices::Servo<configs::UART::Servo, configs::Pins::UART::Servo::Tx, configs::Pins::UART::Servo::Rx> servo;
devices::ECU<configs::UART::ECU, configs::Pins::UART::ECU::Tx, configs::Pins::UART::ECU::Rx> ecu;
devices::Button<configs::Pins::Button> mode_button;
devices::Switch<configs::Pins::Brake> brake_switch;
devices::Switch<configs::Pins::Guard> guard_switch;
devices::Switch<configs::Pins::Clutch> clutch_switch;
devices::Indicator<configs::Pins::Led> mode_indicator;
devices::Indicator<configs::Pins::Led> status_indicator;

Controller controller{logger, accelerator, servo, ecu, mode_button, brake_switch, guard_switch, clutch_switch, mode_indicator, status_indicator};
SystemHost system_host{controller};

}  // namespace

extern "C" void app_main() {
  controller.init();
  system_host.run();
}
