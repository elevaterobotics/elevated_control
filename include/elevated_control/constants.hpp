// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "elevated_control/types.hpp"

namespace elevated_control {

// Physical EtherCAT bus indices
inline constexpr std::size_t kYaw1Idx = 0;
inline constexpr std::size_t kYaw2Idx = 1;
inline constexpr std::size_t kSpringAdjustIdx = 2;
inline constexpr std::size_t kElevationInertialIdx = 3;
inline constexpr std::size_t kWristYawIdx = 4;
inline constexpr std::size_t kWristPitchIdx = 5;
inline constexpr std::size_t kWristRollIdx = 6;

// EtherCAT operation modes
inline constexpr std::uint8_t kProfileTorqueMode = 4;
inline constexpr std::uint8_t kCyclicPositionMode = 8;
inline constexpr std::uint8_t kCyclicVelocityMode = 9;

// Controlword values
inline constexpr std::uint16_t kNormalOpBrakesOff = 0b00001111;
inline constexpr std::uint16_t kQuickStopCtrlWord = 0b00001011;

// Buffer to subtract from position limits (rad)
inline constexpr float kPositionLimitBuffer = 0.05f;
// In torque mode, begin pushing back when this far from the limit (rad)
inline constexpr float kJointLimitTorqueBrakeDistance = 0.09f;

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

// Cyclic loop sleep (microseconds)
inline constexpr unsigned int kCyclicLoopSleepUs = 5000;

// Minimum successful PDO exchanges before joint state readout is trusted (matches
// synapticon_ros2_control::SynapticonSystemInterface::read). At ~5 ms/cycle, 200 ≈ 1 s.
inline constexpr int kMinPdoExchanges = 200;

// Deadband for wrist pitch dial
inline constexpr float kWristPitchDeadband = 0.05f;
inline constexpr float kWristRollDeadband = 0.1f;
// Brake hysteresis for hand-guided pitch
inline constexpr float kWristPitchBrakeOnThreshold = 0.01f;
inline constexpr float kWristPitchBrakeOffThreshold = 0.02f;

// Max wrist velocity from hand-guided dial (rad/s)
inline constexpr float kMaxWristPitchVelocity = 0.3f;
inline constexpr float kMaxWristRollVelocity = 0.6f;

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
