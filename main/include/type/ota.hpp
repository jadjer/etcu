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
// Created by jadjer on 9.09.26.
//

#pragma once

namespace type {

template <std::size_t PayloadSize>
struct OTAChunk {
  std::uint32_t firmware_size{0};
  std::uint16_t chunk_total{0};
  std::uint16_t chunk_index{0};

  std::array<std::uint8_t, PayloadSize> payload{};
};

}  // namespace type
