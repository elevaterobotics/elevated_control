// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025 Synapticon GmbH

#pragma once

#include <cstdint>

namespace elevated_control {

#pragma pack(push, 1)

struct InSomanet50t {
  std::uint16_t Statusword;
  std::int8_t OpModeDisplay;
  std::int32_t PositionValue;
  std::int32_t VelocityValue;
  std::int16_t TorqueValue;
  std::uint16_t AnalogInput1;
  std::uint16_t AnalogInput2;
  std::uint16_t AnalogInput3;
  std::uint16_t AnalogInput4;
  std::uint32_t TuningStatus;
  std::uint32_t DigitalInputs;
  std::uint32_t UserMISO;
  std::uint32_t Timestamp;
  std::int32_t PositionDemandInternalValue;
  std::int32_t VelocityDemandValue;
  std::int16_t TorqueDemand;
};

struct OutSomanet50t {
  std::uint16_t Controlword;
  std::int8_t OpMode;
  std::int16_t TargetTorque;
  std::int32_t TargetPosition;
  std::int32_t TargetVelocity;
  std::int16_t TorqueOffset;
  std::int32_t TuningCommand;
  std::int32_t PhysicalOutputs;
  std::int32_t BitMask;
  std::int32_t UserMOSI;
  std::int32_t VelocityOffset;
};

#pragma pack(pop)

}  // namespace elevated_control
