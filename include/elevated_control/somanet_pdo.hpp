// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025 Synapticon GmbH

#pragma once

#include <cstdint>

namespace elevated_control {

#pragma pack(push, 1)

// TxPDO (drive → host) — 47 bytes
// Mapping: 0x1A00 + 0x1A01 + 0x1A02 + 0x1A03
struct InSomanet50t {
  std::uint16_t Statusword;                 // 0x6041:00  16 bits  byte  0
  std::int8_t OpModeDisplay;                // 0x6061:00   8 bits  byte  2
  std::int32_t PositionValue;               // 0x6064:00  32 bits  byte  3
  std::int32_t VelocityValue;               // 0x606C:00  32 bits  byte  7
  std::int16_t TorqueValue;                 // 0x6077:00  16 bits  byte 11
  std::uint16_t AnalogInput1;               // 0x2401:00  16 bits  byte 13
  std::uint16_t AnalogInput2;               // 0x2402:00  16 bits  byte 15
  std::uint16_t AnalogInput3;               // 0x2403:00  16 bits  byte 17
  std::uint16_t AnalogInput4;               // 0x2404:00  16 bits  byte 19
  std::uint32_t TuningStatus;               // 0x2702:00  32 bits  byte 21
  std::uint32_t DigitalInputs;              // 0x60FD:00  32 bits  byte 25
  std::uint32_t UserMISO;                   // 0x2704:00  32 bits  byte 29
  std::uint32_t Timestamp;                  // 0x20F0:00  32 bits  byte 33
  std::int32_t PositionDemandInternalValue; // 0x60FC:00  32 bits  byte 37
  std::int32_t VelocityDemandValue;         // 0x606B:00  32 bits  byte 41
  std::int16_t TorqueDemand;                // 0x6074:00  16 bits  byte 45
};

// RxPDO (host → drive) — 35 bytes
// Mapping: 0x1600 + 0x1601 + 0x1602
struct OutSomanet50t {
  std::uint16_t Controlword;                // 0x6040:00  16 bits  byte  0
  std::int8_t OpMode;                       // 0x6060:00   8 bits  byte  2
  std::int16_t TargetTorque;                // 0x6071:00  16 bits  byte  3
  std::int32_t TargetPosition;              // 0x607A:00  32 bits  byte  5
  std::int32_t TargetVelocity;              // 0x60FF:00  32 bits  byte  9
  std::int16_t TorqueOffset;                // 0x60B2:00  16 bits  byte 13
  std::int32_t TuningCommand;               // 0x2701:00  32 bits  byte 15
  std::int32_t PhysicalOutputs;             // 0x60FE:01  32 bits  byte 19
  std::int32_t BitMask;                     // 0x60FE:02  32 bits  byte 23
  std::int32_t UserMOSI;                    // 0x2703:00  32 bits  byte 27
  std::int32_t VelocityOffset;              // 0x60B1:00  32 bits  byte 31
};

#pragma pack(pop)

}  // namespace elevated_control
