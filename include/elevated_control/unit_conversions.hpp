// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>

#include "elevated_control/types.hpp"

namespace elevated_control {

// Startup angle wrap handling for single-turn encoders
// TODO: remove this when multi-turn encoders are available
inline std::array<std::once_flag, kNumJoints> startup_angle_wrap_flag{};
inline std::array<std::atomic<float>, kNumJoints> startup_angle_wrap_value{};

// On the first call for each joint, a one-time offset is computed (via std::call_once)
// that brings the initial angle into [-pi, pi]. Handles multi-turn encoder accumulation.
inline float InputTicksToOutputShaftRad(const std::int32_t ticks,
                                        const float mechanical_reduction,
                                        const std::uint32_t encoder_resolution,
                                        const std::size_t joint_idx) {
  float unwrapped_angle =
      (static_cast<float>(ticks) / static_cast<float>(encoder_resolution)) *
      2.0f * static_cast<float>(M_PI) / mechanical_reduction;

  std::call_once(startup_angle_wrap_flag[joint_idx], [&]() {
    float wrapped = std::fmod(unwrapped_angle, 2.0f * static_cast<float>(M_PI));
    if (wrapped > static_cast<float>(M_PI)) {
      wrapped -= 2.0f * static_cast<float>(M_PI);
    } else if (wrapped < -static_cast<float>(M_PI)) {
      wrapped += 2.0f * static_cast<float>(M_PI);
    }
    startup_angle_wrap_value[joint_idx] = wrapped - unwrapped_angle;
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

// Synapticon Velocity actual/demand: 0x60A9 = 0xFDB44700 => 0.001 RPM user units.
// Unknown units fall back to legacy tick-based conversion.
inline float VelocityValueToOutputShaftRadPerS(
    std::int32_t velocity_value, std::int32_t si_velocity_unit,
    float mechanical_reduction, std::uint32_t encoder_resolution) {
  constexpr std::int32_t kMilliRpmUnit =
      static_cast<std::int32_t>(0xFDB44700u);
  if (si_velocity_unit == kMilliRpmUnit) {
    return static_cast<float>(velocity_value) * 2.0f * static_cast<float>(M_PI) /
           (60.0f * 1000.0f);
  }
  return InputTicksVelocityToOutputShaftRadPerS(velocity_value, mechanical_reduction,
                                                encoder_resolution);
}

inline std::int32_t OutputShaftRadPerSToVelocityValue(
    float output_shaft_rad_per_sec, std::int32_t si_velocity_unit,
    float mechanical_reduction, std::uint32_t encoder_resolution) {
  constexpr std::int32_t kMilliRpmUnit =
      static_cast<std::int32_t>(0xFDB44700u);
  if (si_velocity_unit == kMilliRpmUnit) {
    return static_cast<std::int32_t>(output_shaft_rad_per_sec * 60.0f * 1000.0f /
                                     (2.0f * static_cast<float>(M_PI)));
  }
  return OutputShaftRadPerSToInputTicksPerS(output_shaft_rad_per_sec,
                                            mechanical_reduction,
                                            encoder_resolution);
}

inline float SpringPotTicksToPayloadKg(const std::int32_t spring_pot_ticks) {
  return 0.03f * spring_pot_ticks - 5.8f;
}

}  // namespace elevated_control
