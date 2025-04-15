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

#pragma once

#include <i2c/Device.hpp>
#include <i2c/Master.hpp>
#include <foc/encoder/EncoderBase.hpp>

class AS5600 : public foc::EncoderBase {
public:
    using Byte = std::uint8_t;
    using Device = std::unique_ptr<i2c::Device>;
    using BusMaster = std::unique_ptr<i2c::Master>;

public:
    enum PowerMode : std::uint8_t {
        POWER_MODE_NORMAL [[maybe_unused]] = 0b00,
        POWER_MODE_LOW_1 [[maybe_unused]] = 0b01,
        POWER_MODE_LOW_2 [[maybe_unused]] = 0b10,
        POWER_MODE_LOW_3 [[maybe_unused]] = 0b11,
    };

    enum Hysteresis : std::uint8_t {
        HYSTERESIS_OFF [[maybe_unused]] = 0b00,
        HYSTERESIS_LSB [[maybe_unused]] = 0b01,
        HYSTERESIS_2_LSB [[maybe_unused]] = 0b10,
        HYSTERESIS_3_LSB [[maybe_unused]] = 0b11,
    };

    enum OutputStage : std::uint8_t {
        OUTPUT_STAGE_ANALOG_FULL [[maybe_unused]] = 0b00,
        OUTPUT_STAGE_ANALOG_REDUCED [[maybe_unused]] = 0b01,
        OUTPUT_STAGE_DIGITAL_PWM [[maybe_unused]] = 0b10,
    };

    enum PWMFrequency : std::uint8_t {
        FREQUENCY_115_HZ [[maybe_unused]] = 0b00,
        FREQUENCY_230_HZ [[maybe_unused]] = 0b01,
        FREQUENCY_460_HZ [[maybe_unused]] = 0b10,
        FREQUENCY_920_HZ [[maybe_unused]] = 0b11,
    };

    enum SlowFilter : std::uint8_t {
        SLOW_FILTER_X16 [[maybe_unused]] = 0b00,
        SLOW_FILTER_X8 [[maybe_unused]] = 0b01,
        SLOW_FILTER_X4 [[maybe_unused]] = 0b10,
        SLOW_FILTER_X2 [[maybe_unused]] = 0b11,
    };

    enum FastFilterThreshold : std::uint16_t {
        SLOW_FILTER_ONLY [[maybe_unused]] = 0b000,
        FAST_FILTER_THRESHOLD_6_LSB [[maybe_unused]] = 0b001,
        FAST_FILTER_THRESHOLD_7_LSB [[maybe_unused]] = 0b010,
        FAST_FILTER_THRESHOLD_9_LSB [[maybe_unused]] = 0b011,
        FAST_FILTER_THRESHOLD_18_LSB [[maybe_unused]] = 0b100,
        FAST_FILTER_THRESHOLD_21_LSB [[maybe_unused]] = 0b101,
        FAST_FILTER_THRESHOLD_24_LSB [[maybe_unused]] = 0b110,
        FAST_FILTER_THRESHOLD_10_LSB [[maybe_unused]] = 0b111,
    };

    struct __attribute__((__packed__)) Configuration {
        PowerMode powerMode: 2;
        Hysteresis hysteresis: 2;
        OutputStage outputStage: 2;
        PWMFrequency pwmFrequency: 2;
        SlowFilter slowFilter: 2;
        FastFilterThreshold fastFilterThreshold: 3;
        bool watchdog: 1;
        [[maybe_unused]] std::uint8_t reserved: 2;
    };

    struct __attribute__((__packed__)) Status {
        [[maybe_unused]] std::uint8_t reserved_1: 3;
        bool magnetHigh: 1;
        bool magnetLow: 1;
        bool magnetDetected: 1;
        [[maybe_unused]] std::uint8_t reserved_2: 2;
    };

public:
    AS5600();

    ~AS5600() override = default;

public:
    [[maybe_unused]] void setPowerMode(PowerMode powerMode);

    [[maybe_unused]] void setHysteresis(Hysteresis hysteresis);

    [[maybe_unused]] void setOutputStage(OutputStage outputStage);

    [[maybe_unused]] void setPWMFrequency(PWMFrequency pwmFrequency);

    [[maybe_unused]] void setSlowFilter(SlowFilter slowFilter);

    [[maybe_unused]] void setFastFilterThreshold(FastFilterThreshold fastFilterThreshold);

    [[maybe_unused]] void setWatchdog(bool enable);

public:
    Angle getMechanicalAngle() override;

    Angle getAngle() override;

    Angle getPreciseAngle() override;

    Angle getVelocity() override;

    Rotations getFullRotations() override;

public:
    void update() override;

public:
    int needsSearch() override;

protected:
    Angle getSensorAngle() override;

    void init() override;

public:
    [[nodiscard]] AS5600::Status getStatus();

    [[nodiscard]] AS5600::Configuration getConfiguration();

private:
    AS5600::Device m_device = nullptr;
    AS5600::BusMaster m_busMaster = nullptr;

private:
    Status m_status = {};
    Configuration m_configuration = {};
};
