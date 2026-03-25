// Copyright (c) 2025 Elevate Robotics Inc

#include "elevated_control/spring_adjust.hpp"

#include <algorithm>
#include <cmath>

#include "elevated_control/constants.hpp"

namespace elevated_control {

float SpringAdjustByLIPS(float target_position,
                         std::int32_t current_lips_position,
                         bool& allow_mode_change,
                         SpringAdjustState& state) {
  constexpr float kP = 15.0f;
  constexpr float kD = 0.0f;
  float error = target_position - static_cast<float>(current_lips_position);
  auto time_now = std::chrono::steady_clock::now();
  std::chrono::duration<float> time_elapsed = time_now - state.time_prev;
  float error_dt = 0.0f;
  if (state.error_prev) {
    error_dt = (error - *state.error_prev) / time_elapsed.count();
  }
  state.error_prev = error;
  state.time_prev = time_now;
  float actuator_torque = kP * error + kD * error_dt;

  if (actuator_torque > 0) {
    actuator_torque = std::clamp(actuator_torque, kSpringAdjustMinTorque,
                                 kSpringAdjustMaxTorque);
  } else {
    actuator_torque = std::clamp(actuator_torque, -kSpringAdjustMaxTorque,
                                 -kSpringAdjustMinTorque);
  }

  if (std::abs(error) < 50.0f) {
    actuator_torque = 0.0f;
    if (!allow_mode_change) {
      allow_mode_change = true;
    }
  }

  return actuator_torque;
}

}  // namespace elevated_control
