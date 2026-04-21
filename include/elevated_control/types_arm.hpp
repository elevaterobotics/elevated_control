// Copyright (c) 2026 Elevate Robotics Inc

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "elevated_control/types.hpp"

// Extension of basic types for the 7-DOF arm

namespace elevated_control {

inline constexpr std::uint8_t kNumJoints = 7;

// Physical EtherCAT bus order for the 7-DOF arm.
enum class JointName : std::uint8_t {
  kYaw1 = 0,
  kYaw2 = 1,
  kSpringAdjust = 2,
  kElevationInertial = 3,
  kWristYaw = 4,
  kWristPitch = 5,
  kWristRoll = 6,
};

constexpr std::size_t JointIndex(JointName j) noexcept {
  return static_cast<std::size_t>(j);
}

// Arm-specific control modes that extend ControlMode for the 7-DOF manipulator.
enum class ArmControlMode : std::uint8_t {
  kHandGuided = 0,
  kSpringAdjust = 1,
};

// Combined control level for ArmInterface: superset of ControlMode + ArmControlMode.
// Kept for backward compatibility with existing ArmInterface code.
enum class ControlLevel : std::uint8_t {
  kUndefined = 0,
  kHandGuided = 1,
  kVelocity = 2,
  kPosition = 3,
  kTorque = 4,
  kQuickStop = 5,
  kSpringAdjust = 6,
};

// Convert base ControlMode to the arm-specific ControlLevel superset.
inline ControlLevel ToControlLevel(ControlMode mode) {
  switch (mode) {
    case ControlMode::kUndefined:  return ControlLevel::kUndefined;
    case ControlMode::kVelocity:   return ControlLevel::kVelocity;
    case ControlMode::kPosition:   return ControlLevel::kPosition;
    case ControlMode::kTorque:     return ControlLevel::kTorque;
    case ControlMode::kQuickStop:  return ControlLevel::kQuickStop;
  }
  return ControlLevel::kUndefined;
}

// Convert arm ControlLevel to base ControlMode.
// Arm-specific modes (kHandGuided, kSpringAdjust) ultimately drive the
// SOMANET in profile-torque mode, so they map to ControlMode::kTorque. This
// lets the base CiA402 state machine progress drives to Operation Enabled
// and then defer to ArmInterface::OnNormalOperation, which overrides the
// base dispatch for these joints.
inline ControlMode ToControlMode(ControlLevel level) {
  switch (level) {
    case ControlLevel::kUndefined:    return ControlMode::kUndefined;
    case ControlLevel::kVelocity:     return ControlMode::kVelocity;
    case ControlLevel::kPosition:     return ControlMode::kPosition;
    case ControlLevel::kTorque:       return ControlMode::kTorque;
    case ControlLevel::kQuickStop:    return ControlMode::kQuickStop;
    case ControlLevel::kHandGuided:   return ControlMode::kTorque;
    case ControlLevel::kSpringAdjust: return ControlMode::kTorque;
  }
  return ControlMode::kUndefined;
}

inline bool IsBaseControlMode(ControlLevel level) {
  return level != ControlLevel::kHandGuided &&
         level != ControlLevel::kSpringAdjust;
}

// Fixed-size container for per-joint values (EtherCAT bus order).
template <typename T>
using JointArray = std::array<T, kNumJoints>;

using JointFloatArray = JointArray<float>;
using JointBoolArray = JointArray<bool>;
using JointControlLevelArray = JointArray<ControlLevel>;

inline bool IsSpringStreamingCommandSlotUnused(float v) {
  return std::isnan(v);
}

// Joint names in EtherCAT bus order, used for YAML config lookup.
inline constexpr std::array<std::string_view, kNumJoints> kJointNames = {
    "yaw_1_joint",
    "yaw_2_joint",
    "spring_adjust_joint",
    "elevation_inertial_joint",
    "wrist_yaw_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
};

}  // namespace elevated_control
