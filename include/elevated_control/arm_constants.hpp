// Copyright (c) 2025 Elevate Robotics Inc
// Arm-specific constants for the 7-DOF manipulator.

#pragma once

#include <array>
#include <cstdint>

#include "elevated_control/arm_types.hpp"

namespace elevated_control {

// Physical EtherCAT bus indices for the 7-DOF arm
inline constexpr std::size_t kYaw1Idx = 0;
inline constexpr std::size_t kYaw2Idx = 1;
inline constexpr std::size_t kSpringAdjustIdx = 2;
inline constexpr std::size_t kElevationInertialIdx = 3;
inline constexpr std::size_t kWristYawIdx = 4;
inline constexpr std::size_t kWristPitchIdx = 5;
inline constexpr std::size_t kWristRollIdx = 6;

// Extra holding torque for wrist pitch (per-mille of rated torque)
inline constexpr float kWristPitchHoldTorque = 100.0f;

// Multiplier for Drake feedforward torque
inline constexpr float kTorqueNmToPerMilleMultiplier = 0.001f;

// Friction compensation per joint (per-mille)
inline constexpr std::array<std::int16_t, kNumJoints> kTorqueFrictionOffset = {
    250, 50, 0, 0, 3, 0, 0};
inline constexpr float kFrictionDeadband = 0.001f;  // rad/s
inline constexpr float kYawAlignmentFrictionThreshold = 0.3f;  // rad

// Max brake torque per joint near limits (per-mille)
inline constexpr std::array<float, kNumJoints> kMaxJointLimitBrakeTorque = {
    250.f, 450.f, 250.f, 250.f, 400.f, 250.f, 250.f};

// Minimum allowable spring position (potentiometer ticks)
inline constexpr float kMinAllowableSpringPosition = 18000.f;

// Wrist dials (hand-guided): normalized input [-1, 1] after centering.
inline constexpr float kMaxWristPitchDialVelocity = 0.3f;  // rad/s
inline constexpr float kWristPitchDeadband = 0.2f;
inline constexpr float kWristPitchDeadbandHysteresis =
    0.1f;  // should remain < kWristPitchDeadband
inline constexpr float kMaxWristRollDialVelocity = 0.6f;  // rad/s
inline constexpr float kWristRollDeadband = 0.1f;

// Admittance velocity scaling (matches synapticon ADMITTANCE_VELOCITY_MULTIPLIER)
inline constexpr float kAdmittanceVelocityMultiplier = 10000.0f;

// Wrist yaw soft limits for hand-guided mode (human-safe)
inline constexpr float kWristYawHandGuidedSoftLimit = 2.5f;         // rad
inline constexpr float kWristYawHandGuidedBrakeRamp = 0.2f;         // rad
inline constexpr float kWristYawHandGuidedMaxBrakeTorque = 80.0f;  // per-mill

// Spring adjust torque bounds (per-mille of rated torque)
inline constexpr float kSpringAdjustMinTorque = 1000.0f;
inline constexpr float kSpringAdjustMaxTorque = 3000.0f;

}  // namespace elevated_control
