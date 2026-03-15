// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <vector>

#include "elevated_control/types.hpp"

namespace elevated_control {

// Startup angle wrap handling for single-turn encoders
// TODO: remove this when multi-turn encoders are available
inline std::vector<std::once_flag> startup_angle_wrap_flag(kNumJoints);
inline std::vector<std::atomic<float>> startup_angle_wrap_value(kNumJoints);

inline float InputTicksToOutputShaftRad(const std::int32_t ticks,
                                        const float mechanical_reduction,
                                        const std::uint32_t encoder_resolution,
                                        const std::size_t joint_idx) {
  float unwrapped_angle =
      (static_cast<float>(ticks) / static_cast<float>(encoder_resolution)) *
      2.0f * static_cast<float>(M_PI) / mechanical_reduction;

  std::call_once(startup_angle_wrap_flag[joint_idx], [&]() {
    if (unwrapped_angle > static_cast<float>(M_PI)) {
      startup_angle_wrap_value[joint_idx] = -2.0f * static_cast<float>(M_PI);
    } else if (unwrapped_angle < -static_cast<float>(M_PI)) {
      startup_angle_wrap_value[joint_idx] = 2.0f * static_cast<float>(M_PI);
    }
  });

  return unwrapped_angle + startup_angle_wrap_value[joint_idx];
}

inline float InputTicksVelocityToOutputShaftRadPerS(
    std::int32_t ticks, float mechanical_reduction,
    std::uint32_t encoder_resolution) {
  return (static_cast<float>(ticks) / encoder_resolution) * 2.0f *
         static_cast<float>(M_PI) / mechanical_reduction;
}

inline std::int32_t OutputShaftRadToInputTicks(
    float output_shaft_rad, float mechanical_reduction,
    std::uint32_t encoder_resolution) {
  return static_cast<std::int32_t>(output_shaft_rad * encoder_resolution *
                                   mechanical_reduction /
                                   (2.0f * static_cast<float>(M_PI)));
}

inline std::int32_t OutputShaftRadToInputTicks(
    float output_shaft_rad, float mechanical_reduction,
    std::uint32_t encoder_resolution, std::size_t joint_idx) {
  float unwrapped_rad =
      output_shaft_rad - startup_angle_wrap_value[joint_idx];
  return static_cast<std::int32_t>(unwrapped_rad * encoder_resolution *
                                   mechanical_reduction /
                                   (2.0f * static_cast<float>(M_PI)));
}

inline std::int16_t TorqueNmToTorquePerMille(
    const float torque_nm, const std::atomic<float>& mechanical_reduction,
    const std::atomic<float>& rated_torque) {
  return static_cast<std::int16_t>(torque_nm * 1000.0f /
                                   (mechanical_reduction * rated_torque));
}

inline float TorquePerMilleToTorqueNm(
    const std::int16_t torque_per_mille,
    const std::atomic<float>& mechanical_reduction,
    const std::atomic<float>& rated_torque) {
  return torque_per_mille * mechanical_reduction * rated_torque / 1000.0f;
}

inline std::int32_t OutputShaftRadPerSToInputTicksPerS(
    float output_shaft_rad_per_sec, float mechanical_reduction,
    std::uint32_t encoder_resolution) {
  return static_cast<std::int32_t>(output_shaft_rad_per_sec *
                                   encoder_resolution * mechanical_reduction /
                                   (2.0f * static_cast<float>(M_PI)));
}

inline float SpringPotTicksToPayloadKg(const std::int32_t spring_pot_ticks) {
  return 0.03f * spring_pot_ticks - 5.8f;
}

}  // namespace elevated_control
