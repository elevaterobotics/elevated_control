// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <cstdint>
#include <string>

#include "elevated_control/arm_types.hpp"

namespace elevated_control {

struct JointLimitsConfig {
  bool valid = false;
  JointBoolArray has_position_limits{};
  JointFloatArray min_position_limits{};
  JointFloatArray max_position_limits{};
};

struct DialConfig {
  std::int32_t min = 0;
  std::int32_t center = 0;
  std::int32_t max = 0;
};

struct ButtonConfig {
  DialConfig roll_dial;
  DialConfig pitch_dial;
};

struct SpringSetpointsConfig {
  std::int32_t spring_setpoint_unloaded = 0;
  std::int32_t spring_setpoint_loaded = 0;
};

struct TorqueSignConfig {
  std::int32_t yaw1 = 1;
  std::int32_t yaw2 = 1;
  std::int32_t wrist_yaw = 1;
};

struct PositionSignConfig {
  std::int32_t inertial = 1;
  std::int32_t wrist_pitch = 1;
};

struct VelocitySignConfig {
  std::int32_t inertial = 1;
  std::int32_t wrist_pitch = 1;
};

struct ElevateConfig {
  ButtonConfig button_config;
  SpringSetpointsConfig spring_setpoints;
  TorqueSignConfig torque_sign;
  PositionSignConfig position_sign;
  VelocitySignConfig velocity_sign;
};

// Parse joint limits from YAML (fixed kNumJoints entries, bus order in kJointNames).
JointLimitsConfig ParseJointLimits(const std::string& joint_limits_file);

ElevateConfig ParseElevateConfig(const std::string& elevate_config_file);

bool ValidateJointLimits(const JointLimitsConfig& config);
bool ValidateDialConfig(const DialConfig& config, const std::string& name);
bool ValidateButtonConfig(const ButtonConfig& config);
bool ValidateSpringSetpoints(const SpringSetpointsConfig& config);
bool ValidateTorqueSignsConfig(const TorqueSignConfig& config);
bool ValidatePositionSignsConfig(const PositionSignConfig& config);
bool ValidateVelocitySignsConfig(const VelocitySignConfig& config);

}  // namespace elevated_control
