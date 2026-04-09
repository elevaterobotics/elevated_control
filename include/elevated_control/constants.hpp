// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <cstddef>
#include <cstdint>

namespace elevated_control {

// EtherCAT operation modes
inline constexpr std::uint8_t kProfileTorqueMode = 4;
inline constexpr std::uint8_t kCyclicPositionMode = 8;
inline constexpr std::uint8_t kCyclicVelocityMode = 9;

// Controlword values
inline constexpr std::uint16_t kNormalOpBrakesOff = 0b00001111;
inline constexpr std::uint16_t kQuickStopCtrlWord = 0b00001011;

// Buffer to subtract from position limits, allowing the robot to reverse out of
// the position limit (rad)
inline constexpr float kPositionLimitBuffer = 0.3f;
static_assert(kPositionLimitBuffer > 0.0f, "Position limit buffer must be positive");
// In torque mode, begin pushing back when this far from the limit (rad)
inline constexpr float kJointLimitTorqueBrakeDistance = 0.09f;

// Cyclic loop sleep (microseconds)
inline constexpr unsigned int kCyclicLoopSleepUs = 5000;

// Minimum successful PDO exchanges before joint state readout is trusted.
// At ~5 ms/cycle, 200 ~ 1 s.
inline constexpr int kMinPdoExchanges = 200;

// Expected EtherCAT slave device name
inline constexpr std::string_view kExpectedSlaveName = "SOMANET";

}  // namespace elevated_control
