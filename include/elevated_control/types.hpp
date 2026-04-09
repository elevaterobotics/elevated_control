// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace elevated_control {

// Generic control modes valid for any number of Synapticon motor drivers.
enum class ControlMode : std::uint8_t {
  kUndefined = 0,
  kVelocity = 1,
  kPosition = 2,
  kTorque = 3,
  kQuickStop = 4,
};

enum class ErrorCode : std::uint8_t {
  kSuccess = 0,
  kNotInitialized,
  kAlreadyInitialized,
  kEtherCATError,
  kInvalidArgument,
  kInvalidSpringSetpoint,
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
