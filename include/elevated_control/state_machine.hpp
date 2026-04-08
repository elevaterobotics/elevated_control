// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <spdlog/spdlog.h>

#include "elevated_control/constants.hpp"
#include "elevated_control/somanet_pdo.hpp"
#include "elevated_control/types.hpp"

namespace elevated_control {

// Bring a single joint to a stop, optionally applying brakes.
inline void Stop(OutSomanet50t* out, bool apply_brake) {
  out->TargetTorque = 0;
  out->TorqueOffset = 0;
  out->TargetVelocity = 0;
  out->VelocityOffset = 0;
  out->OpMode = kCyclicVelocityMode;
  if (apply_brake) {
    out->Controlword = 0;
  }
}

// SDO read helpers (wrap ec_SDOread from SOEM)
std::optional<std::int32_t> ReadSDOValue(std::uint16_t slave,
                                         std::uint16_t index,
                                         std::uint8_t subindex);

std::optional<std::string> ReadSDOString(std::uint16_t slave,
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

inline void HandleShutdown(OutSomanet50t* out_somanet,
                           ControlMode control_mode,
                           bool mode_switch_in_progress,
                           bool hold_in_shutdown) {
  Stop(out_somanet, true);

  if (!hold_in_shutdown &&
      (control_mode != ControlMode::kQuickStop) &&
      (control_mode != ControlMode::kUndefined) &&
      !mode_switch_in_progress) {
    out_somanet->Controlword = 0b00000110;
  }
}

inline void HandleSwitchOn(OutSomanet50t* out_somanet,
                           bool hold_in_shutdown) {
  if (hold_in_shutdown) {
    Stop(out_somanet, true);
    return;
  }
  out_somanet->Controlword = 0b00000111;
}

inline void HandleEnableOperation(OutSomanet50t* out_somanet,
                                  bool mode_switch_in_progress,
                                  bool hold_in_shutdown) {
  if (hold_in_shutdown) {
    Stop(out_somanet, true);
    return;
  }
  if (!mode_switch_in_progress) {
    out_somanet->Controlword = kNormalOpBrakesOff;
  }
}

}  // namespace elevated_control
