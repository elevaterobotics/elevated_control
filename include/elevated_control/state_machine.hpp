// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include <spdlog/spdlog.h>

#include "elevated_control/constants.hpp"
#include "elevated_control/somanet_pdo.hpp"
#include "elevated_control/types.hpp"

namespace elevated_control {

// Bring the robot to a stop, optionally applying brakes.
// If brakes are applied, the EtherCAT state machine transitions to
// "Switch on disabled".
inline void Stop(std::array<OutSomanet50t*, kNumJoints>& out_somanet,
                 bool apply_brake, std::size_t joint_idx) {
  out_somanet[joint_idx]->TargetTorque = 0;
  out_somanet[joint_idx]->TorqueOffset = 0;
  out_somanet[joint_idx]->TargetVelocity = 0;
  out_somanet[joint_idx]->VelocityOffset = 0;
  out_somanet[joint_idx]->OpMode = kCyclicVelocityMode;
  if (apply_brake) {
    out_somanet[joint_idx]->Controlword = 0;
  }
}

// SDO read helper (wraps ec_SDOread from SOEM)
std::optional<std::int32_t> ReadSDOValue(std::uint16_t slave,
                                         std::uint16_t index,
                                         std::uint8_t subindex);

inline void HandleFault(const InSomanet50t* in_somanet,
                        OutSomanet50t* out_somanet, std::size_t joint_idx) {
  out_somanet->Controlword = 0b10000000;
  auto fault_code = ReadSDOValue(
      static_cast<std::uint16_t>(joint_idx + 1), 0x603F, 0x00);
  spdlog::error(
      "EtherCAT state fault on joint {}, TorqueDemand: {}, "
      "VelocityDemandValue: {}, FaultCode: 0x{:x}",
      joint_idx, in_somanet->TorqueDemand, in_somanet->VelocityDemandValue,
      fault_code.value_or(-1));
}

// Handle "Switch on disabled" state: stop and brake, manage pitch dial logic
inline void HandleShutdown(std::array<OutSomanet50t*, kNumJoints>& out_somanet,
                           std::size_t joint_idx,
                           const JointControlLevelArray& control_level,
                           bool mode_switch_in_progress) {
  Stop(out_somanet, true, joint_idx);

  if ((control_level[joint_idx] != ControlLevel::kQuickStop) &&
      (control_level[joint_idx] != ControlLevel::kUndefined) &&
      !mode_switch_in_progress) {
    out_somanet[joint_idx]->Controlword = 0b00000110;
  }
}

inline void HandleSwitchOn(std::array<OutSomanet50t*, kNumJoints>& out_somanet,
                           std::size_t joint_idx) {
  out_somanet[joint_idx]->Controlword = 0b00000111;
}

inline void HandleEnableOperation(
    std::array<OutSomanet50t*, kNumJoints>& out_somanet,
    std::size_t joint_idx, bool mode_switch_in_progress,
    ControlLevel control_level, bool deadman_pressed) {
  if (!mode_switch_in_progress) {
    if (control_level == ControlLevel::kHandGuided && !deadman_pressed) {
      // Operation not enabled yet
      out_somanet[joint_idx]->Controlword = 0b00000111;
    } else {
      out_somanet[joint_idx]->Controlword = kNormalOpBrakesOff;
    }
  }
}

}  // namespace elevated_control
