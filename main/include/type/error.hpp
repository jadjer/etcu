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

static constexpr std::uint32_t ErrorShiftECU{0};
static constexpr std::uint32_t ErrorShiftGuard{6};
static constexpr std::uint32_t ErrorShiftServo{7};
static constexpr std::uint32_t ErrorShiftBluetooth{14};
static constexpr std::uint32_t ErrorShiftIndicator{19};
static constexpr std::uint32_t ErrorShiftPeripheral{20};
static constexpr std::uint32_t ErrorShiftAccelerator{21};

enum class SystemError : std::uint32_t {
  None = 0,

  // ECU (Электронный блок управления)
  EcuInitFailed = 1 << (ErrorShiftECU + 0),
  EcuReadFailed = 1 << (ErrorShiftECU + 1),
  EcuWriteFailed = 1 << (ErrorShiftECU + 2),
  EcuVoltageFailed = 1 << (ErrorShiftECU + 3),
  EcuOverVoltage = 1 << (ErrorShiftECU + 4),
  EcuEngineOverheat = 1 << (ErrorShiftECU + 5),

  // Guard (Защита / Блокировка)
  GuardLocked = 1 << (ErrorShiftGuard + 0),

  // Servo (Сервопривод)
  ServoInitFailed = 1 << (ErrorShiftServo + 0),
  ServoReadFailed = 1 << (ErrorShiftServo + 1),
  ServoWriteFailed = 1 << (ErrorShiftServo + 2),
  ServoEncoderFailed = 1 << (ErrorShiftServo + 3),
  ServoVoltageFailed = 1 << (ErrorShiftServo + 4),
  ServoOverheat = 1 << (ErrorShiftServo + 5),
  ServoOverload = 1 << (ErrorShiftServo + 6),

  // Bluetooth
  BluetoothInitFailed = 1 << (ErrorShiftBluetooth + 0),
  BluetoothPowerFailed = 1 << (ErrorShiftBluetooth + 1),
  BluetoothMtuFailed = 1 << (ErrorShiftBluetooth + 2),
  BluetoothConnFailed = 1 << (ErrorShiftBluetooth + 3),
  BluetoothSendFailed = 1 << (ErrorShiftBluetooth + 4),

  // Indicator (Индикация)
  IndicatorInitFailed = 1 << (ErrorShiftIndicator + 0),

  // Peripheral (Периферия)
  PeripheralInitFailed = 1 << (ErrorShiftPeripheral + 0),

  // Accelerator (Акселератор / Ручка газа)
  AcceleratorInitFailed = 1 << (ErrorShiftAccelerator + 0),
  AcceleratorReadFailed = 1 << (ErrorShiftAccelerator + 1),
  AcceleratorMismatch = 1 << (ErrorShiftAccelerator + 2),
};

[[nodiscard]] constexpr auto operator|(SystemError const a, SystemError const b) -> SystemError {
  return static_cast<SystemError>(static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b));
}

[[nodiscard]] constexpr auto operator&(SystemError const a, SystemError const b) noexcept -> SystemError {
  return static_cast<SystemError>(static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b));
}

[[nodiscard]] constexpr auto operator~(SystemError const a) noexcept -> SystemError {
  return static_cast<SystemError>(~static_cast<std::uint32_t>(a));
}

[[nodiscard]] constexpr auto has_error(SystemError const err) -> bool {
  return static_cast<std::uint32_t>(err) != 0;
}

[[nodiscard]] constexpr auto has_error(SystemError const mask, SystemError const err) -> bool {
  return (static_cast<std::uint32_t>(mask) & static_cast<std::uint32_t>(err)) != 0;
}

[[nodiscard]] constexpr auto make_mask(std::uint32_t const current_shift, std::uint32_t const next_shift) noexcept -> SystemError {
  std::uint32_t const bit_count = next_shift - current_shift;
  std::uint32_t const raw_mask = (1U << bit_count) - 1U;
  return static_cast<SystemError>(raw_mask << current_shift);
}

static constexpr SystemError ErrorMaskECU = make_mask(ErrorShiftECU, ErrorShiftGuard);
static constexpr SystemError ErrorMaskGuard = make_mask(ErrorShiftGuard, ErrorShiftServo);
static constexpr SystemError ErrorMaskServo = make_mask(ErrorShiftServo, ErrorShiftBluetooth);
static constexpr SystemError ErrorMaskBluetooth = make_mask(ErrorShiftBluetooth, ErrorShiftIndicator);
static constexpr SystemError ErrorMaskIndicator = make_mask(ErrorShiftIndicator, ErrorShiftPeripheral);
static constexpr SystemError ErrorMaskPeripheral = make_mask(ErrorShiftPeripheral, ErrorShiftAccelerator);
static constexpr SystemError ErrorMaskAccelerator = make_mask(ErrorShiftAccelerator, 32U);

static_assert((ErrorMaskECU & ErrorMaskGuard) == SystemError::None);
static_assert((ErrorMaskGuard & ErrorMaskServo) == SystemError::None);
static_assert((ErrorMaskServo & ErrorMaskBluetooth) == SystemError::None);
static_assert((ErrorMaskBluetooth & ErrorMaskIndicator) == SystemError::None);
static_assert((ErrorMaskIndicator & ErrorMaskPeripheral) == SystemError::None);
static_assert((ErrorMaskPeripheral & ErrorMaskAccelerator) == SystemError::None);

static_assert(has_error(ErrorMaskECU, SystemError::EcuEngineOverheat));
static_assert(has_error(ErrorMaskGuard, SystemError::GuardLocked));
static_assert(has_error(ErrorMaskServo, SystemError::ServoOverload));
static_assert(has_error(ErrorMaskBluetooth, SystemError::BluetoothSendFailed));
static_assert(has_error(ErrorMaskIndicator, SystemError::IndicatorInitFailed));
static_assert(has_error(ErrorMaskPeripheral, SystemError::PeripheralInitFailed));
static_assert(has_error(ErrorMaskAccelerator, SystemError::AcceleratorMismatch));

}  // namespace type
