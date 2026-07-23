//
// Created by jadjer on 23.07.26.
//

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "configs/configs.hpp"

template <concepts::ControllerConcept Controller>
class SystemHost {
  Controller& m_controller;

  [[noreturn]] static auto critical_task_adapter(void* pvParameters) -> void {
    auto* host = static_cast<SystemHost*>(pvParameters);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
      host->m_controller.process_critical_loop();
      vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
  }

  [[noreturn]] static auto system_task_adapter(void* pvParameters) -> void {
    auto* host = static_cast<SystemHost*>(pvParameters);

    while (true) {
      host->m_controller.process_system_loop();
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }

 public:
  explicit SystemHost(Controller& controller) noexcept : m_controller(controller) {}

  SystemHost(SystemHost const&) = delete;
  auto operator=(SystemHost const&) -> SystemHost& = delete;

  auto run() -> void {
    xTaskCreatePinnedToCore(&SystemHost::system_task_adapter, "SystemTask", 4096, this, 5, nullptr,
                            configs::System::SystemCore);
    xTaskCreatePinnedToCore(&SystemHost::critical_task_adapter, "CriticalTask", 4096, this, 10,
                            nullptr, configs::System::CriticalCore);
  }
};
