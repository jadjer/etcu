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

#include <esp_log.h>

#include "App.hpp"

namespace {

auto const TAG = "App";

} // namespace

extern "C" void app_main() {
  auto app = App::create();
  if (not app) {
    ESP_LOGE(TAG, "System init failed");
    return;
  }

  if (not(*app)->setup()) {
    ESP_LOGE(TAG, "Components init failed");
    return;
  }

  (*app)->run();
}
