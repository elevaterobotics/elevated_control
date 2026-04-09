// Copyright (c) 2026 Elevate Robotics Inc

#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <spdlog/spdlog.h>

namespace elevated_control {

// These functions assume the velocity encoder is on the motor shaft,
// the position encoder is on the output shaft.

// Compute raw angle from encoder ticks. Wrap state parameters allow per-instance
// startup wrapping (previously kept in global arrays).
inline float InputTicksToOutputShaftRad(const std::int32_t ticks,
                                        const float mechanical_reduction,
                                        const std::uint32_t encoder_resolution,
                                        const std::atomic<float>& wrap_value) {
  float unwrapped_angle =
      (static_cast<float>(ticks) / static_cast<float>(encoder_resolution)) *
      2.0f * static_cast<float>(M_PI) / mechanical_reduction;
  return unwrapped_angle + wrap_value.load(std::memory_order_relaxed);
}

// Compute the one-time startup wrap offset and store it.
// Call once per joint at startup (typically via std::call_once).
inline void ComputeStartupWrapOffset(std::int32_t ticks,
                                     float mechanical_reduction,
                                     std::uint32_t encoder_resolution,
                                     std::atomic<float>& wrap_value) {
  float unwrapped_angle =
      (static_cast<float>(ticks) / static_cast<float>(encoder_resolution)) *
      2.0f * static_cast<float>(M_PI) / mechanical_reduction;
  float wrapped = std::fmod(unwrapped_angle, 2.0f * static_cast<float>(M_PI));
  if (wrapped > static_cast<float>(M_PI)) {
    wrapped -= 2.0f * static_cast<float>(M_PI);
  } else if (wrapped < -static_cast<float>(M_PI)) {
    wrapped += 2.0f * static_cast<float>(M_PI);
  }
  wrap_value.store(wrapped - unwrapped_angle, std::memory_order_relaxed);
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
    std::uint32_t encoder_resolution,
    const std::atomic<float>& wrap_value) {
  float unwrapped_rad =
      output_shaft_rad - wrap_value.load(std::memory_order_relaxed);
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

// SDO 0x60A9 SI velocity unit encoding (CiA DS-303):
//   Top byte = signed power-of-10 exponent, lower 3 bytes = RPM base unit.
//   0xFDB44700 => exponent -3 => 0.001 RPM (milli-RPM)
//   0x00B44700 => exponent  0 => 1 RPM
constexpr std::int32_t kMilliRpmUnit = static_cast<std::int32_t>(0xFDB44700u);
constexpr std::int32_t kRpmUnit = static_cast<std::int32_t>(0x00B44700u);

// Values from the drive are motor-shaft speed; divide by mechanical_reduction
// to obtain output-shaft rad/s. Unknown units fall back to legacy tick-based conversion.
inline float VelocityValueToOutputShaftRadPerS(
    std::int32_t velocity_value, std::int32_t si_velocity_unit,
    float mechanical_reduction, std::uint32_t encoder_resolution) {
  // Milli-RPM
  if (si_velocity_unit == kMilliRpmUnit) {
    return static_cast<float>(velocity_value) * 2.0f * static_cast<float>(M_PI) /
           (60.0f * 1000.0f * mechanical_reduction);
  }
  // RPM
  else if (si_velocity_unit == kRpmUnit) {
    return static_cast<float>(velocity_value) * 2.0f * static_cast<float>(M_PI) /
           (60.0f * mechanical_reduction);
  }
  // Tick-based conversion
  return InputTicksVelocityToOutputShaftRadPerS(velocity_value, mechanical_reduction,
                                                encoder_resolution);
}

// Convert output-shaft rad/s to motor RPM or milli-RPM.
inline std::int32_t OutputShaftRadPerSToVelocityValue(
    float output_shaft_rad_per_sec, std::int32_t si_velocity_unit) {
  if (si_velocity_unit == kMilliRpmUnit) {
    // Convert rad/s to milliRPM, mechanical_reduction is automatically applied in the drive
    return static_cast<std::int32_t>(std::lround(output_shaft_rad_per_sec *
                                                60.0f * 1000.0f /
                                                (2.0f * static_cast<float>(M_PI))));
  }
  if (si_velocity_unit == kRpmUnit) {
    // Convert rad/s to RPM, mechanical_reduction is automatically applied in the drive
    return static_cast<std::int32_t>(std::lround(output_shaft_rad_per_sec * 60.0f /
                                                 (2.0f * static_cast<float>(M_PI))));
  }
  spdlog::error(
      "OutputShaftRadPerSToVelocityValue: unhandled si_velocity_unit {} "
      "(supported: RPM [{}] or milliRPM [{}])",
      si_velocity_unit, kRpmUnit, kMilliRpmUnit);
  return 0;
}

inline float SpringPotTicksToPayloadKg(const std::int32_t spring_pot_ticks) {
  return 0.03f * spring_pot_ticks - 5.8f;
}

}  // namespace elevated_control
