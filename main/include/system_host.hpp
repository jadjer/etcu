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
#include <types.hpp>
#include "concepts.hpp"

template <concepts::ControllerConcept Controller, CoreId SystemCore = 0, CoreId CriticalCore = 1, CoreRate Rate = 10>
  requires concepts::CoreIdConcept<SystemCore> && concepts::CoreIdConcept<CriticalCore> && concepts::CoreRateConcept<Rate>
class SystemHost {
  Controller& m_controller;

  [[noreturn]] static auto system_task_adapter(void* pvParameters) -> void {
    auto* host = static_cast<SystemHost*>(pvParameters);

    while (true) {
      host->m_controller.process_system_loop();
      vTaskDelay(pdMS_TO_TICKS(Rate));
    }
  }

  [[noreturn]] static auto critical_task_adapter(void* pvParameters) -> void {
    auto* host = static_cast<SystemHost*>(pvParameters);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
      host->m_controller.process_critical_loop();
      vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(Rate));
    }
  }

 public:
  explicit SystemHost(Controller& controller) noexcept : m_controller(controller) {}

  SystemHost(SystemHost const&) = delete;
  auto operator=(SystemHost const&) -> SystemHost& = delete;

  auto run() -> void {
    xTaskCreatePinnedToCore(&SystemHost::system_task_adapter, "SystemTask", 4096, this, 5, nullptr, SystemCore);
    xTaskCreatePinnedToCore(&SystemHost::critical_task_adapter, "CriticalTask", 4096, this, 10, nullptr, CriticalCore);
  }
};
