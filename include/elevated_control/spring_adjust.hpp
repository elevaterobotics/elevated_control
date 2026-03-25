// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>

#include "elevated_control/types.hpp"

namespace elevated_control {

struct SpringAdjustState {
  std::chrono::steady_clock::time_point time_prev;
  std::optional<float> error_prev;
};

// PD spring tracking toward `target_position` (potentiometer ticks). When
// |error| is below the completion threshold, torque is zeroed and
// `allow_mode_change` is set true (first transition only while false).
float SpringAdjustByLIPS(float target_position,
                         std::int32_t current_lips_position,
                         bool& allow_mode_change,
                         SpringAdjustState& state);

// Clears the spring setpoint latch and transitions the joint to quick stop
// after `SpringAdjustByLIPS` has signaled completion (`allow_mode_change`
// true). The next spring-adjust session requires a new `SetSpringSetpoint`.
inline void CompleteSpringAdjustSession(
    std::optional<std::atomic<float>>& spring_setpoint_target,
    ControlLevel& joint_control_level) {
  spring_setpoint_target = std::nullopt;
  joint_control_level = ControlLevel::kQuickStop;
}

}  // namespace elevated_control
