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
// Created by jadjer on 28.07.26.
//

#pragma once

#include <NimBLEServer.h>
#include <atomic>

namespace bluetooth::callback {

class ServerCallback : public NimBLEServerCallbacks {
  std::atomic<bool> m_connected{false};

 public:
  constexpr ServerCallback() noexcept = default;

  ServerCallback(ServerCallback const&) noexcept = delete;
  auto operator=(ServerCallback const&) noexcept -> ServerCallback& = delete;

  ServerCallback(ServerCallback&&) noexcept = delete;
  auto operator=(ServerCallback&&) noexcept -> ServerCallback& = delete;

  ~ServerCallback() noexcept override = default;

  auto onConnect(NimBLEServer*, NimBLEConnInfo&) -> void override { m_connected.store(true, std::memory_order_relaxed); }

  auto onDisconnect(NimBLEServer* server, NimBLEConnInfo&, int) -> void override {
    m_connected.store(false, std::memory_order_relaxed);

    std::ignore = server->startAdvertising();
  }

  [[nodiscard]] auto isConnected() const -> bool { return m_connected.load(std::memory_order_relaxed); }
};

}  // namespace bluetooth::callback
