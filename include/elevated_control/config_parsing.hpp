// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace elevated_control {

struct JointLimitsConfig {
  std::vector<bool> has_position_limits;
  std::vector<float> min_position_limits;
  std::vector<float> max_position_limits;
};

struct ButtonConfig {
  std::int32_t analog_input_midpoint = 0;
  std::int32_t wrist_dial_min = 0;
  std::int32_t wrist_dial_max = 0;
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

struct ElevateConfig {
  ButtonConfig button_config;
  SpringSetpointsConfig spring_setpoints;
  TorqueSignConfig torque_sign;
};

// Parse joint limits from YAML. joint_names lists the joint names in bus order.
JointLimitsConfig ParseJointLimits(const std::string& joint_limits_file,
                                   const std::vector<std::string>& joint_names);

ElevateConfig ParseElevateConfig(const std::string& elevate_config_file);

bool ValidateJointLimits(const JointLimitsConfig& config);
bool ValidateButtonConfig(const ButtonConfig& config);
bool ValidateSpringSetpoints(const SpringSetpointsConfig& config);
bool ValidateTorqueSignsConfig(const TorqueSignConfig& config);

}  // namespace elevated_control
