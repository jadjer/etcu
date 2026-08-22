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
#include "concepts.hpp"
#include "type.hpp"

template <class Controller, type::CoreId systemCore = 0, type::CoreId criticalCore = 1, type::CoreRate rate = 10>
  requires concepts::Controller<Controller> && concepts::CoreId<systemCore> && concepts::CoreId<criticalCore> && concepts::CoreRate<rate>

class SystemHost {
  Controller& m_controller;

  [[noreturn]] static auto system_task_adapter(void* pv_parameters) -> void {
    auto* host = static_cast<SystemHost*>(pv_parameters);

    while (true) {
      host->m_controller.process_system_loop();
      vTaskDelay(pdMS_TO_TICKS(rate));
    }
  }

  [[noreturn]] static auto critical_task_adapter(void* pv_parameters) -> void {
    auto* host = static_cast<SystemHost*>(pv_parameters);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
      host->m_controller.process_critical_loop();
      vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(rate));
    }
  }

 public:
  constexpr explicit SystemHost(Controller& controller) noexcept : m_controller(controller) {}

  auto run() -> void {
    xTaskCreatePinnedToCore(&SystemHost::system_task_adapter, "SystemTask", 4096, this, 5, nullptr, systemCore);
    xTaskCreatePinnedToCore(&SystemHost::critical_task_adapter, "CriticalTask", 4096, this, 10, nullptr, criticalCore);
  }
};
