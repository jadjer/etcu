//
// Created by jadjer on 4.09.26.
//

#pragma once

namespace device {

enum class ServoRegister : std::uint8_t {
  // =========================================================================
  // ЗОНА EEPROM (Энергонезависимая. Запись доступна только при RegLockSign = 0)
  // =========================================================================
  FormwareMajorVersion = 0x00,
  FormwareMinorVersion = 0x01,
  ServoMajorVersion = 0x03,
  ServoMinorVersion = 0x04,
  Id = 0x05,
  BaudRate = 0x06,
  ReturnDelayTime = 0x07,
  ResponseStatusLevel = 0x08,
  MinAngleLimit = 0x09,
  MaxAngleLimit = 0x0B,
  MaxTemperatureLimit = 0x0D,
  MaxInputVoltageLimit = 0x0E,
  MinInputVoltageLimit = 0x0F,
  MaxTorqueLimit = 0x10,
  Phase = 0x12,
  UnloadCondition = 0x13,
  LedAlarmCondition = 0x14,
  ComplianceP = 0x15,
  ComplianceD = 0x16,
  ComplianceI = 0x17,
  MinStartingForce = 0x18,
  CWDeadZone = 0x1A,
  CCWDeadZone = 0x1B,
  ProtectionCurrent = 0x1C,
  AngleResolution = 0x1E,
  PositionCorrection = 0x1F,
  OperationMode = 0x21,
  ProtectionTorque = 0x22,
  ProtectionTime = 0x23,
  OverloadTorque = 0x24,
  SpeedCloseLoopP = 0x25,
  OvercurrentProtectionTime = 0x26,
  VelocityCloseLoopI = 0x27,

  // =========================================================================
  // ЗОНА SRAM (Оперативная. Сбрасывается в дефолт при выключении питания)
  // =========================================================================
  TorqueEnable = 0x28,
  Acceleration = 0x29,
  TargetPosition = 0x2A,
  OperationTime = 0x2C,
  OperationSpeed = 0x2E,
  TorqueLimit = 0x30,

  LockFlag = 0x37,
  CurrentPosition = 0x38,
  CurrentSpeed = 0x3A,
  CurrentLoad = 0x3C,
  CurrentVoltage = 0x3E,
  CurrentTemperature = 0x3F,
  AsynchronousWriteFlag = 0x40,
  ServoStatus = 0x41,
  MoveFlag = 0x42,

  CurrentCurrent = 0x45,
};

template <typename T>
  requires std::is_enum_v<T> || std::convertible_to<T, std::uint8_t>
[[nodiscard]] constexpr auto as_byte(T const& value) noexcept -> std::uint8_t {
  return static_cast<std::uint8_t>(value) & 0xFF;
}

}
