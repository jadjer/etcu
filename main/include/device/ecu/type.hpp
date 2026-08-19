//
// Created by jadjer on 18.08.26.
//

#pragma once

namespace device::type {

using Byte = std::uint8_t;

struct EngineData {
  uint16_t rpm;
  uint16_t fuelInject;
  uint8_t ignitionAdvance;
  uint8_t unkData1;
  uint8_t unkData2;
  uint8_t unkData3;
};

struct ErrorData {};

struct SensorsData {
  float tpsPercent;
  float tpsVolts;
  uint8_t ectTemp;
  float ectVolts;
  uint8_t iatTemp;
  float iatVolts;
  uint8_t mapPressure;
  float mapVolts;
};

struct UnknownData {
  uint8_t unkData1;
  uint8_t unkData2;
  uint8_t unkData3;
  uint8_t unkData4;
  uint8_t unkData5;
  uint8_t unkData6;
  uint8_t unkData7;
  uint8_t unkData8;
  uint8_t unkData9;
  uint8_t unkData10;
  uint8_t unkData11;
  uint8_t unkData12;
  uint8_t unkData13;
  uint8_t unkData14;
  uint8_t unkData15;
  uint8_t unkData16;
  uint8_t unkData17;
  uint8_t unkData18;
  uint8_t unkData19;
  uint8_t unkData20;
  uint8_t unkData21;
  uint8_t unkData22;
  uint8_t unkData23;
  uint8_t unkData24;
  uint8_t unkData25;
  uint8_t unkData26;
  uint8_t unkData27;
  uint8_t unkData28;
  uint8_t unkData29;
  uint8_t unkData30;
  uint8_t unkData31;
  uint8_t unkData32;
  uint8_t unkData33;
  uint8_t unkData34;
  uint8_t unkData35;
  uint8_t unkData36;
  uint8_t unkData37;
  uint8_t unkData38;
  uint8_t unkData39;
  uint8_t unkData40;
};

struct VehicleData {
  std::string id;
  float batteryVolts;
  uint8_t speed;
  uint8_t state;
};

struct CommandResult {
  uint8_t code;
  uint8_t command;
  uint8_t length;
  uint8_t checksum;
  uint8_t* data;
};

}
