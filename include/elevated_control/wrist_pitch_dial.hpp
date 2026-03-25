// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include "elevated_control/constants.hpp"

namespace elevated_control {

/**
 * @brief Map post-deadband normalized dial [-1, 1] to commanded wrist pitch velocity (rad/s).
 *
 * Piecewise linear between ±kWristPitchDeadband and ±1, saturates at ±kMaxWristPitchVelocity
 * outside ±1. Values strictly inside (-kWristPitchDeadband, kWristPitchDeadband) yield 0
 * (typically the caller zeros the dial in the deadband before calling this).
 */
inline float WristPitchDialNormalizedToVelocity(float normalized_dial) {
  const float v_min = -kMinWristPitchDialVelocity;
  const float v_max = kMaxWristPitchVelocity;
  const float slope =
      (v_max - v_min) / (1.0f - kWristPitchDeadband);
  if (normalized_dial > 1.0f) {
    return v_max;
  }
  if ((normalized_dial >= kWristPitchDeadband) && (normalized_dial <= 1.0f)) {
    return slope * (normalized_dial - kWristPitchDeadband) + v_min;
  }
  if (normalized_dial < -1.0f) {
    return -v_max;
  }
  if ((normalized_dial <= -kWristPitchDeadband) &&
      (normalized_dial >= -1.0f)) {
    return slope * (normalized_dial - (-kWristPitchDeadband)) - v_min;
  }
  return 0.0f;
}

}  // namespace elevated_control
