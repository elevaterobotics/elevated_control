// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>

#include "elevated_control/types.hpp"

namespace elevated_control {

// Converts commanded spring load (N) to LIPS potentiometer tick target.
// Used by `ArmInterface::SetSpringSetpoint`.
inline float ConvertNewtonsToSpringLipsTicks(float load_newtons) {
  constexpr float kSlope = 3.7852f;
  constexpr float kIntercept = 161.1f;
  return kSlope * load_newtons + kIntercept;
}

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

// Threadsafe spring setpoint storage
inline void StoreSpringAdjustSetpoint(
    std::atomic<float>& spring_setpoint_target_ticks,
    std::atomic<bool>& has_spring_setpoint,
    float target_ticks) {
  spring_setpoint_target_ticks.store(target_ticks, std::memory_order_relaxed);
  has_spring_setpoint.store(true, std::memory_order_release);
}

// Threadsafe spring setpoint retrieval
inline std::optional<float> LoadSpringAdjustSetpoint(
    const std::atomic<float>& spring_setpoint_target_ticks,
    const std::atomic<bool>& has_spring_setpoint) {
  if (!has_spring_setpoint.load(std::memory_order_acquire)) {
    return std::nullopt;
  }
  return spring_setpoint_target_ticks.load(std::memory_order_relaxed);
}

// Clears the spring setpoint latch and transitions the joint to quick stop
// after `SpringAdjustByLIPS` has signaled completion (`allow_mode_change`
// true). The next spring-adjust session requires a new `SetSpringSetpoint`.
inline void CompleteSpringAdjustSession(
    std::atomic<bool>& has_spring_setpoint,
    ControlLevel& joint_control_level) {
  has_spring_setpoint.store(false, std::memory_order_release);
  joint_control_level = ControlLevel::kQuickStop;
}

}  // namespace elevated_control
