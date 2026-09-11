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
// Created by jadjer on 24.08.26.
//

#pragma once

namespace type {

static constexpr std::uint16_t ErrorShiftECU{0};
static constexpr std::uint16_t ErrorShiftGuard{2};
static constexpr std::uint16_t ErrorShiftServo{3};
static constexpr std::uint16_t ErrorShiftBluetooth{6};
static constexpr std::uint16_t ErrorShiftIndicator{11};
static constexpr std::uint16_t ErrorShiftPeripheral{12};
static constexpr std::uint16_t ErrorShiftAccelerator{13};

enum class SystemError : std::uint16_t {
  None = 0,
  ECUInitFault = 1 << (ErrorShiftECU + 0),                    // 0
  ECUReadError = 1 << (ErrorShiftECU + 1),                    // 1
  GuardLock = 1 << (ErrorShiftGuard + 0),                     // 2
  ServoInitError = 1 << (ErrorShiftServo + 0),                // 3
  ServoReadError = 1 << (ErrorShiftServo + 1),                // 4
  ServoWriteError = 1 << (ErrorShiftServo + 2),               // 5
  BluetoothInitFault = 1 << (ErrorShiftBluetooth + 0),        // 6
  BluetoothSetPowerFault = 1 << (ErrorShiftBluetooth + 1),    // 7
  BluetoothSetMTUFault = 1 << (ErrorShiftBluetooth + 2),      // 8
  BluetoothConnectedFault = 1 << (ErrorShiftBluetooth + 3),   // 9
  BluetoothSendNotifyError = 1 << (ErrorShiftBluetooth + 4),  // 10
  IndicatorInitFault = 1 << (ErrorShiftIndicator + 0),        // 11
  PeripheralInitError = 1 << (ErrorShiftPeripheral + 0),      // 12
  AcceleratorInitError = 1 << (ErrorShiftAccelerator + 0),    // 13
  AcceleratorReadError = 1 << (ErrorShiftAccelerator + 1),    // 14
  AcceleratorMismatch = 1 << (ErrorShiftAccelerator + 2),     // 15
};

[[nodiscard]] constexpr auto operator|(SystemError const a, SystemError const b) -> SystemError {
  return static_cast<SystemError>(static_cast<std::uint16_t>(a) | static_cast<std::uint16_t>(b));
}

[[nodiscard]] constexpr auto operator&(SystemError const a, SystemError const b) noexcept -> SystemError {
  return static_cast<SystemError>(static_cast<std::uint16_t>(a) & static_cast<std::uint16_t>(b));
}

[[nodiscard]] constexpr auto operator~(SystemError const a) noexcept -> SystemError {
  return static_cast<SystemError>(~static_cast<std::uint16_t>(a));
}

[[nodiscard]] constexpr auto has_error(SystemError const err) -> bool {
  return static_cast<std::uint16_t>(err) != 0;
}

[[nodiscard]] constexpr auto has_error(SystemError const mask, SystemError const err) -> bool {
  return (static_cast<std::uint16_t>(mask) & static_cast<std::uint16_t>(err)) != 0;
}

[[nodiscard]] constexpr auto make_mask(std::uint16_t const current_shift, std::uint16_t const next_shift) noexcept -> SystemError {
  std::uint16_t const bit_count = next_shift - current_shift;
  std::uint16_t const raw_mask = (1U << bit_count) - 1U;
  return static_cast<SystemError>(raw_mask << current_shift);
}

static constexpr SystemError ErrorMaskECU = make_mask(ErrorShiftECU, ErrorShiftGuard);
static constexpr SystemError ErrorMaskGuard = make_mask(ErrorShiftGuard, ErrorShiftServo);
static constexpr SystemError ErrorMaskServo = make_mask(ErrorShiftServo, ErrorShiftBluetooth);
static constexpr SystemError ErrorMaskBluetooth = make_mask(ErrorShiftBluetooth, ErrorShiftIndicator);
static constexpr SystemError ErrorMaskIndicator = make_mask(ErrorShiftIndicator, ErrorShiftPeripheral);
static constexpr SystemError ErrorMaskPeripheral = make_mask(ErrorShiftPeripheral, ErrorShiftAccelerator);
static constexpr SystemError ErrorMaskAccelerator = make_mask(ErrorShiftAccelerator, 16U);

static_assert((ErrorMaskECU & ErrorMaskGuard) == SystemError::None);
static_assert((ErrorMaskGuard & ErrorMaskServo) == SystemError::None);
static_assert((ErrorMaskServo & ErrorMaskBluetooth) == SystemError::None);
static_assert((ErrorMaskBluetooth & ErrorMaskIndicator) == SystemError::None);
static_assert((ErrorMaskIndicator & ErrorMaskPeripheral) == SystemError::None);
static_assert((ErrorMaskPeripheral & ErrorMaskAccelerator) == SystemError::None);

static_assert(has_error(ErrorMaskECU, SystemError::ECUReadError));
static_assert(has_error(ErrorMaskGuard, SystemError::GuardLock));
static_assert(has_error(ErrorMaskServo, SystemError::ServoWriteError));
static_assert(has_error(ErrorMaskBluetooth, SystemError::BluetoothConnectedFault));
static_assert(has_error(ErrorMaskIndicator, SystemError::IndicatorInitFault));
static_assert(has_error(ErrorMaskPeripheral, SystemError::PeripheralInitError));
static_assert(has_error(ErrorMaskAccelerator, SystemError::AcceleratorMismatch));

}  // namespace type
