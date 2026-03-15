// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <cstdint>
#include <string>

namespace elevated_control {

inline constexpr std::uint8_t kNumJoints = 7;

// Physical EtherCAT bus order
enum class JointName : std::uint8_t {
  kYaw1 = 0,
  kYaw2 = 1,
  kSpringAdjust = 2,
  kElevationInertial = 3,
  kWristYaw = 4,
  kWristPitch = 5,
  kWristRoll = 6,
};

enum class ControlLevel : std::uint8_t {
  kUndefined = 0,
  kHandGuided = 1,
  kVelocity = 2,
  kPosition = 3,
  kTorque = 4,
  kQuickStop = 5,
  kSpringAdjust = 6,
};

enum class ErrorCode : std::uint8_t {
  kSuccess = 0,
  kNotInitialized,
  kAlreadyInitialized,
  kEtherCATError,
  kInvalidArgument,
  kInvalidMode,
  kModeChangeDenied,
  kJointLimitViolation,
  kTimeout,
  kControlLoopNotRunning,
  kControlLoopAlreadyRunning,
  kHalted,
};

struct Error {
  ErrorCode code;
  std::string message;
};

}  // namespace elevated_control
