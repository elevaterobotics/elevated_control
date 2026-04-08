#pragma once

#include <cmath>

#include "elevated_control/arm_constants.hpp"

namespace elevated_control {

struct DialActivationState {
  bool is_active = false;

  void Reset() { is_active = false; }
};

// Returns true while the pitch dial remains latched active.
inline bool UpdateWristPitchDialActivationState(
    float normalized_dial, DialActivationState& state) {
  const float abs_normalized_dial = std::abs(normalized_dial);
  float release_threshold = 0.0f;
  if (state.is_active) {
    release_threshold =
        kWristPitchDeadband - kWristPitchDeadbandHysteresis;
  }

  if (!state.is_active) {
    state.is_active = abs_normalized_dial >= kWristPitchDeadband;
  } else if (abs_normalized_dial <= release_threshold) {
    state.is_active = false;
  }
  return state.is_active;
}

inline float NormalizeWristPitchDialToVelocity(float normalized_dial) {
  const float v_min = 0.0f;
  const float v_max = kMaxWristPitchDialVelocity;
  const float slope = (v_max - v_min) / (1.0f - kWristPitchDeadband);
  if (normalized_dial > 1.0f) {
    return v_max;
  }
  if (normalized_dial >= kWristPitchDeadband) {
    return slope * (normalized_dial - kWristPitchDeadband) + v_min;
  }
  if (normalized_dial < -1.0f) {
    return -v_max;
  }
  if (normalized_dial <= -kWristPitchDeadband) {
    return slope * (normalized_dial + kWristPitchDeadband) - v_min;
  }
  return 0.0f;
}

inline float NormalizeWristRollDialToVelocity(float normalized_dial) {
  float velocity = 0.0f;
  const float slope =
      kMaxWristRollDialVelocity / (1.0f - kWristRollDeadband);
  if (normalized_dial > 1.0f) {
    velocity = kMaxWristRollDialVelocity;
  } else if (normalized_dial >= kWristRollDeadband) {
    velocity = slope * (normalized_dial - kWristRollDeadband);
  } else if (normalized_dial < -1.0f) {
    velocity = -kMaxWristRollDialVelocity;
  } else if (normalized_dial <= -kWristRollDeadband) {
    velocity = slope * (normalized_dial + 1.0f) - kMaxWristRollDialVelocity;
  }
  return velocity;
}

}  // namespace elevated_control
