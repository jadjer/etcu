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

namespace constants {

auto constexpr MAX_BLE_PAYLOAD_SIZE = 244;

auto constexpr DEVICE_NAME = "ETCU";

auto constexpr SERVICE_UUID = "019fa351-08ac-76bf-b925-fe3ae2f765fb";

auto constexpr OTA_CHARACTERISTIC_UUID = "019fa351-08ac-7d45-8718-b4aa5af6756a";
auto constexpr CONTROL_CHARACTERISTIC_UUID = "019fa351-08ac-7309-804b-ad328e7c1ef1";
auto constexpr TELEMETRY_CHARACTERISTIC_UUID = "019fa351-08ac-7940-a519-6ef5087c0329";

} // namespace constants
