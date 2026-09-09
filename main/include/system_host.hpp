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
  { controller.process_ecu_loop() } noexcept -> std::same_as<void>;
  { controller.process_ota_loop() } noexcept -> std::same_as<void>;
  { controller.process_system_loop() } noexcept -> std::same_as<void>;
  { controller.process_control_loop() } noexcept -> std::same_as<void>;
  { controller.process_critical_loop() } noexcept -> std::same_as<void>;
  { controller.process_telemetry_loop() } noexcept -> std::same_as<void>;
  { controller.process_calibration_loop() } noexcept -> std::same_as<void>;
};

template <class Controller, std::uint8_t SystemCore = 0, std::uint8_t CriticalCore = 1>
  requires ControllerConcept<Controller> && (SystemCore <= 1) && (CriticalCore <= 1)

class SystemHost {
  Controller& m_controller;

  [[noreturn]] static auto ecu_task_adapter(void* parameters) -> void {
    auto* host = static_cast<SystemHost*>(parameters);

    while (true) {
      host->m_controller.process_ecu_loop();
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  [[noreturn]] static auto ota_task_adapter(void* parameters) -> void {
    auto* host = static_cast<SystemHost*>(parameters);

    while (true) {
      host->m_controller.process_ota_loop();
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  [[noreturn]] static auto control_task_adapter(void* parameters) -> void {
    auto* host = static_cast<SystemHost*>(parameters);

    while (true) {
      host->m_controller.process_control_loop();
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  [[noreturn]] static auto telemetry_task_adapter(void* parameters) -> void {
    auto* host = static_cast<SystemHost*>(parameters);

    while (true) {
      host->m_controller.process_telemetry_loop();
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  [[noreturn]] static auto calibration_task_adapter(void* parameters) -> void {
    auto* host = static_cast<SystemHost*>(parameters);

    while (true) {
      host->m_controller.process_calibration_loop();
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  [[noreturn]] static auto system_task_adapter(void* parameters) -> void {
    auto* host = static_cast<SystemHost*>(parameters);

    while (true) {
      host->m_controller.process_system_loop();
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  [[noreturn]] static auto critical_task_adapter(void* parameters) -> void {
    auto* host = static_cast<SystemHost*>(parameters);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
      host->m_controller.process_critical_loop();
      vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
  }

 public:
  constexpr explicit SystemHost(Controller& controller) noexcept : m_controller(controller) {}

  constexpr SystemHost() noexcept = delete;

  SystemHost(SystemHost const&) noexcept = delete;
  auto operator=(SystemHost const&) noexcept -> SystemHost& = delete;

  SystemHost(SystemHost&&) noexcept = delete;
  auto operator=(SystemHost&&) noexcept -> SystemHost& = delete;

  constexpr ~SystemHost() noexcept = default;

  auto run() -> void {
    xTaskCreatePinnedToCore(&SystemHost::ecu_task_adapter, "ECUTask", 4096, this, 0, nullptr, SystemCore);
    xTaskCreatePinnedToCore(&SystemHost::ota_task_adapter, "OTATask", 4096, this, 0, nullptr, SystemCore);
    xTaskCreatePinnedToCore(&SystemHost::control_task_adapter, "ControlTask", 4096, this, 0, nullptr, SystemCore);
    xTaskCreatePinnedToCore(&SystemHost::telemetry_task_adapter, "TelemetryTask", 4096, this, 0, nullptr, SystemCore);
    xTaskCreatePinnedToCore(&SystemHost::calibration_task_adapter, "CalibrationTask", 4096, this, 0, nullptr, SystemCore);

    xTaskCreatePinnedToCore(&SystemHost::system_task_adapter, "SystemTask", 4096, this, 5, nullptr, SystemCore);

    xTaskCreatePinnedToCore(&SystemHost::critical_task_adapter, "CriticalTask", 4096, this, 10, nullptr, CriticalCore);
  }
};
