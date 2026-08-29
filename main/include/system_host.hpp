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

//
// Created by jadjer on 23.07.26.
//

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

template <typename T>
concept ControllerConcept = requires(T controller) {
  { controller.init() } noexcept -> std::same_as<void>;
  { controller.process_ota_loop() } noexcept -> std::same_as<void>;
  { controller.process_system_loop() } noexcept -> std::same_as<void>;
  { controller.process_critical_loop() } noexcept -> std::same_as<void>;
};

template <class Controller, std::uint8_t SystemCore = 0, std::uint8_t CriticalCore = 1, std::uint16_t RateMS = 10>
  requires ControllerConcept<Controller> && (SystemCore <= 1) && (CriticalCore <= 1) && (RateMS >= 10)

class SystemHost {
  Controller& m_controller;

  [[noreturn]] static auto system_task_adapter(void* parameters) -> void {
    auto* host = static_cast<SystemHost*>(parameters);

    while (true) {
      host->m_controller.process_system_loop();
      host->m_controller.process_ota_loop();
      vTaskDelay(pdMS_TO_TICKS(RateMS));
    }
  }

  [[noreturn]] static auto critical_task_adapter(void* parameters) -> void {
    auto* host = static_cast<SystemHost*>(parameters);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
      host->m_controller.process_critical_loop();
      vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(RateMS));
    }
  }

 public:
  constexpr explicit SystemHost(Controller& controller) noexcept : m_controller(controller) {}

  auto run() -> void {
    xTaskCreatePinnedToCore(&SystemHost::system_task_adapter, "SystemTask", 4096, this, 1, nullptr, SystemCore);
    xTaskCreatePinnedToCore(&SystemHost::critical_task_adapter, "CriticalTask", 4096, this, 10, nullptr, CriticalCore);
  }
};
