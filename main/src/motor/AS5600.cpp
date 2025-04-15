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

#include "motor/AS5600.hpp"

#include <algorithm>
#include <cstring>
#include <esp_log.h>

auto const TAG = "AS5600";

auto const DEVICE_ADDRESS = 0x36;    ///< Адрес устройства
auto const REGISTER_ZMCO = 0x00;     ///< Количество записей
auto const REGISTER_ZPOS = 0x01;     ///< Минимальное положение
auto const REGISTER_MPOS = 0x03;     ///< Максимальное положение
auto const REGISTER_MANG = 0x05;     ///< Диапазон значений
auto const REGISTER_CONF = 0x07;     ///<  Конфигурация
auto const REGISTER_STATUS = 0x0B;   ///< Статус
auto const REGISTER_RAW_ANGLE = 0x0C;///< Угол (сырое значение)
auto const REGISTER_ANGLE = 0x0E;    ///< Угол (значение после фильтрации)
auto const REGISTER_AGC = 0x1A;      ///< Automatic Gain Control
auto const REGISTER_MAGNITUDE = 0x1B;///< Магнитуда
auto const REGISTER_BURN = 0xFF;     ///< Запись в ПЗУ

template<typename T>
T convertBytesToData(std::vector<std::uint8_t> bytes) {
  T struct_{};

  std::reverse(bytes.begin(), bytes.end());
  std::memcpy(&struct_, bytes.data(), bytes.size());

  return struct_;
}

template<typename T>
std::vector<uint8_t> convertDataToBytes(T struct_) {
  std::vector<uint8_t> bytes(sizeof(T));

  std::memcpy(bytes.data(), &struct_, sizeof(T));
  std::reverse(bytes.begin(), bytes.end());

  return bytes;
}

AS5600::AS5600() {
  m_busMaster = std::make_unique<i2c::Master>(21, 22, 0);
  m_device = m_busMaster->createDevice(DEVICE_ADDRESS);

  m_status = getStatus();
  ESP_LOGI(TAG, "Magnet low: %d", m_status.magnetLow);
  ESP_LOGI(TAG, "Magnet high: %d", m_status.magnetHigh);
  ESP_LOGI(TAG, "Magnet detected: %d", m_status.magnetDetected);

  m_configuration = getConfiguration();
  ESP_LOGI(TAG, "Power mode: %d", m_configuration.powerMode);
  ESP_LOGI(TAG, "Hysteresis: %d", m_configuration.hysteresis);
  ESP_LOGI(TAG, "Output stage: %d", m_configuration.outputStage);
  ESP_LOGI(TAG, "PWM Frequency: %d", m_configuration.pwmFrequency);
  ESP_LOGI(TAG, "Slow filter: %d", m_configuration.slowFilter);
  ESP_LOGI(TAG, "Fast filter threshold: %d", m_configuration.fastFilterThreshold);
  ESP_LOGI(TAG, "Watchdog: %d", m_configuration.watchdog);
}

[[maybe_unused]] void AS5600::setPowerMode(AS5600::PowerMode powerMode) {
  m_configuration.powerMode = powerMode;

  auto const configurationData = convertDataToBytes(m_configuration);

  m_device->write(REGISTER_CONF, configurationData);
}

[[maybe_unused]] void AS5600::setHysteresis(AS5600::Hysteresis hysteresis) {
  m_configuration.hysteresis = hysteresis;

  auto const configurationData = convertDataToBytes(m_configuration);

  m_device->write(REGISTER_CONF, configurationData);
}

[[maybe_unused]] void AS5600::setOutputStage(AS5600::OutputStage outputStage) {
  m_configuration.outputStage = outputStage;

  auto const configurationData = convertDataToBytes(m_configuration);

  m_device->write(REGISTER_CONF, configurationData);
}

[[maybe_unused]] void AS5600::setPWMFrequency(AS5600::PWMFrequency pwmFrequency) {
  m_configuration.pwmFrequency = pwmFrequency;

  auto const configurationData = convertDataToBytes(m_configuration);

  m_device->write(REGISTER_CONF, configurationData);
}

[[maybe_unused]] void AS5600::setSlowFilter(AS5600::SlowFilter slowFilter) {
  m_configuration.slowFilter = slowFilter;

  auto const configurationData = convertDataToBytes(m_configuration);

  m_device->write(REGISTER_CONF, configurationData);
}

[[maybe_unused]] void AS5600::setFastFilterThreshold(AS5600::FastFilterThreshold fastFilterThreshold) {
  m_configuration.fastFilterThreshold = fastFilterThreshold;

  auto const configurationData = convertDataToBytes(m_configuration);

  m_device->write(REGISTER_CONF, configurationData);
}

[[maybe_unused]] void AS5600::setWatchdog(bool enable) {
  m_configuration.watchdog = enable;

  auto const configurationData = convertDataToBytes(m_configuration);

  m_device->write(REGISTER_CONF, configurationData);
}

AS5600::Status AS5600::getStatus() {
  if (not m_device) {
    return m_status;
  }

  auto const statusData = m_device->read(REGISTER_STATUS);

  m_status = convertBytesToData<AS5600::Status>(statusData);

  return m_status;
}

AS5600::Configuration AS5600::getConfiguration() {
  if (not m_device) {
    return m_configuration;
  }

  auto const configurationData = m_device->read(REGISTER_CONF, 2);

  m_configuration = convertBytesToData<AS5600::Configuration>(configurationData);

  return m_configuration;
}

float AS5600::getMechanicalAngle() {
  return EncoderBase::getMechanicalAngle();
}

float AS5600::getAngle() {
  return EncoderBase::getAngle();
}

double AS5600::getPreciseAngle() {
  return EncoderBase::getPreciseAngle();
}

float AS5600::getVelocity() {
  return EncoderBase::getVelocity();
}

int32_t AS5600::getFullRotations() {
  return EncoderBase::getFullRotations();
}

void AS5600::update() {
  EncoderBase::update();
}

int AS5600::needsSearch() {
  return EncoderBase::needsSearch();
}

float AS5600::getSensorAngle() {
  if (not m_device) {
    return {};
  }

  auto const angleData = m_device->read(REGISTER_ANGLE, 2);

  return convertBytesToData<AS5600::Angle>(angleData);
}

void AS5600::init() {
  EncoderBase::init();
}
