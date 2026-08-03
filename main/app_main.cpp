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
#include "logger.hpp"
#include "system_host.hpp"
#include "configs/configs.hpp"

namespace {

Logger logger;

devices::Accelerator<configs::ADC::Unit, configs::ADC::Hall1, configs::ADC::Hall2, configs::ADC::MismatchThreshold> accelerator;
devices::Servo<configs::UART::Servo::Port, configs::UART::Servo::Tx, configs::UART::Servo::Rx> servo;
devices::ECU<configs::UART::ECU::Port, configs::UART::ECU::Tx, configs::UART::ECU::Rx> ecu;
devices::Button<configs::Pins::ModeButton, true> mode_button;
devices::Button<configs::Pins::Brake> brake_switch;
devices::Button<configs::Pins::Guard> guard_switch;
devices::Button<configs::Pins::Clutch> clutch_switch;
devices::Indicator<configs::Pins::ModeLed> mode_indicator;
devices::Indicator<configs::Pins::StatusLed> status_indicator;

Controller controller{logger, accelerator, servo, ecu, mode_button, brake_switch, guard_switch, clutch_switch, mode_indicator, status_indicator};

SystemHost<decltype(controller), configs::System::SystemCore, configs::System::CriticalCore> system_host{controller};

}  // namespace

extern "C" void app_main() {
  controller.init();

  system_host.run();
}
