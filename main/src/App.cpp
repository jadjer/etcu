// Copyright 2025 Pavel Suprunov
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

#include "App.hpp"

#include <utility>

#include "Controller.hpp"

namespace {

auto const TAG = "App";

} // namespace

auto App::create() -> std::expected<App::Pointer, App::Error> {
  auto configuration = Configuration::create();
  if (not configuration) {
    return std::unexpected(App::Error::CONFIGURATION_ERROR);
  }

  return App::Pointer(new App(*configuration));
}

App::App(Configuration::Pointer const &configuration) noexcept
    : executor(std::make_unique<executor::Executor>()), telemetry(std::make_shared<Telemetry>()), bluetooth(std::make_unique<Bluetooth>(configuration)),
      configuration(configuration) {}

auto App::setup() -> std::expected<void, App::Error> {
  auto controller = Controller::create(configuration, telemetry);
  if (not controller) {
    return std::unexpected(App::Error::CONTROLLER_INIT_ERROR);
  }

  executor->addNode(std::move(*controller));

  return {};
}

auto App::run() -> void {
  bluetooth->advertise();
  executor->spin();
}
