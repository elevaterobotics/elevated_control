// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <atomic>
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

enum class ModeChangeBlockReason {
  kSpringAdjust,
  kEstop,
  kHandGuidedDeadman,
  kStuckFunctionEnable,
};

// Independent latches that can block normal controller mode switches.
// Each subsystem sets/clears its own reason without disturbing others.
// Should never block a transition to QUICK_STOP mode.
struct PreventModeChange {
  std::atomic<bool> blocked_by_spring_adjust{false};
  std::atomic<bool> blocked_by_estop{false};
  std::atomic<bool> blocked_by_hand_guided{false};
  std::atomic<bool> blocked_by_stuck_function_enable{false};

  bool IsBlocked() const {
    return blocked_by_spring_adjust.load() || blocked_by_estop.load() ||
           blocked_by_hand_guided.load() ||
           blocked_by_stuck_function_enable.load();
  }

  bool AllowsNormalModeSwitch() const { return !IsBlocked(); }

  bool IsBlocked(ModeChangeBlockReason reason) const {
    switch (reason) {
      case ModeChangeBlockReason::kSpringAdjust:
        return blocked_by_spring_adjust.load();
      case ModeChangeBlockReason::kEstop:
        return blocked_by_estop.load();
      case ModeChangeBlockReason::kHandGuidedDeadman:
        return blocked_by_hand_guided.load();
      case ModeChangeBlockReason::kStuckFunctionEnable:
        return blocked_by_stuck_function_enable.load();
    }
    return false;
  }

  void SetBlocked(ModeChangeBlockReason reason) {
    SetBlockedImpl(reason, true);
  }

  void ClearBlocked(ModeChangeBlockReason reason) {
    SetBlockedImpl(reason, false);
  }

  std::string BlockingReasonsString() const {
    std::string reasons;
    const auto append = [&reasons](const std::string& r) {
      if (!reasons.empty()) reasons += ", ";
      reasons += r;
    };
    if (blocked_by_spring_adjust.load()) append("spring_adjust");
    if (blocked_by_estop.load()) append("estop");
    if (blocked_by_hand_guided.load()) append("hand_guided");
    if (blocked_by_stuck_function_enable.load()) append("stuck_function_enable");
    return reasons.empty() ? "none" : reasons;
  }

 private:
  void SetBlockedImpl(ModeChangeBlockReason reason, bool blocked) {
    switch (reason) {
      case ModeChangeBlockReason::kSpringAdjust:
        blocked_by_spring_adjust = blocked;
        break;
      case ModeChangeBlockReason::kEstop:
        blocked_by_estop = blocked;
        break;
      case ModeChangeBlockReason::kHandGuidedDeadman:
        blocked_by_hand_guided = blocked;
        break;
      case ModeChangeBlockReason::kStuckFunctionEnable:
        blocked_by_stuck_function_enable = blocked;
        break;
    }
  }
};

}  // namespace elevated_control
