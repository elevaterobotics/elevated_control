// Copyright (c) 2025 Elevate Robotics Inc

#include "elevated_control/config_parsing.hpp"

#include <cmath>
#include <limits>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

namespace elevated_control {

JointLimitsConfig ParseJointLimits(
    const std::string& joint_limits_file,
    const std::vector<std::string>& joint_names) {
  JointLimitsConfig config;

  try {
    YAML::Node yaml_config = YAML::LoadFile(joint_limits_file);
    if (!yaml_config["joint_limits"]) {
      spdlog::error("No 'joint_limits' section found in {}", joint_limits_file);
      return config;
    }

    YAML::Node joint_limits = yaml_config["joint_limits"];

    config.has_position_limits.resize(joint_names.size(), false);
    config.min_position_limits.resize(
        joint_names.size(), -std::numeric_limits<float>::max());
    config.max_position_limits.resize(
        joint_names.size(), std::numeric_limits<float>::max());

    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      const std::string& joint_name = joint_names[i];

      if (joint_limits[joint_name]) {
        YAML::Node joint_config = joint_limits[joint_name];

        if (joint_config["has_position_limits"]) {
          config.has_position_limits[i] =
              joint_config["has_position_limits"].as<bool>();
        }

        if (config.has_position_limits[i]) {
          if (joint_config["min_position"]) {
            config.min_position_limits[i] =
                joint_config["min_position"].as<float>();
          }
          if (joint_config["max_position"]) {
            config.max_position_limits[i] =
                joint_config["max_position"].as<float>();
          }
          if ((config.min_position_limits[i] >=
               config.max_position_limits[i]) ||
              !std::isfinite(config.min_position_limits[i]) ||
              !std::isfinite(config.max_position_limits[i])) {
            spdlog::error("Invalid position limits for joint {}", joint_name);
            return JointLimitsConfig{};
          }

          spdlog::info("Joint {}: position limits [{:.3f}, {:.3f}]",
                       joint_name, config.min_position_limits[i],
                       config.max_position_limits[i]);
        } else {
          spdlog::info("Joint {}: no position limits", joint_name);
        }
      } else {
        spdlog::warn(
            "Joint {}: no configuration found in joint_limits.yaml",
            joint_name);
      }
    }
  } catch (const YAML::Exception& e) {
    spdlog::error("Failed to parse joint limits YAML: {}", e.what());
    return JointLimitsConfig{};
  } catch (const std::exception& e) {
    spdlog::error("Failed to load joint limits file: {}", e.what());
    return JointLimitsConfig{};
  }

  return config;
}

ElevateConfig ParseElevateConfig(const std::string& elevate_config_file) {
  ElevateConfig config;

  try {
    YAML::Node elevate_config = YAML::LoadFile(elevate_config_file);

    if (!elevate_config["button_config"]) {
      spdlog::error("No 'button_config' section found in {}",
                    elevate_config_file);
      return ElevateConfig{};
    }

    YAML::Node button_config = elevate_config["button_config"];

    if (!button_config["analog_input_midpoint"]) {
      spdlog::error("Missing 'analog_input_midpoint' in button config");
      return ElevateConfig{};
    }
    config.button_config.analog_input_midpoint =
        button_config["analog_input_midpoint"].as<std::int32_t>();

    if (!button_config["wrist_dial_min"]) {
      spdlog::error("Missing 'wrist_dial_min' in elevate config");
      return ElevateConfig{};
    }
    config.button_config.wrist_dial_min =
        button_config["wrist_dial_min"].as<std::int32_t>();

    if (!button_config["wrist_dial_max"]) {
      spdlog::error("Missing 'wrist_dial_max' in elevate config");
      return ElevateConfig{};
    }
    config.button_config.wrist_dial_max =
        button_config["wrist_dial_max"].as<std::int32_t>();

    if (!elevate_config["spring_setpoints"]) {
      spdlog::error("Missing 'spring_setpoints' in elevate yaml");
      return ElevateConfig{};
    }

    if (!elevate_config["spring_setpoints"]["unloaded"]) {
      spdlog::error("Missing 'unloaded' in elevate config");
      return ElevateConfig{};
    }
    config.spring_setpoints.spring_setpoint_unloaded =
        elevate_config["spring_setpoints"]["unloaded"].as<std::int32_t>();

    if (!elevate_config["spring_setpoints"]["loaded"]) {
      spdlog::error("Missing 'loaded' in elevate config");
      return ElevateConfig{};
    }
    config.spring_setpoints.spring_setpoint_loaded =
        elevate_config["spring_setpoints"]["loaded"].as<std::int32_t>();

    if (!elevate_config["torque_sign"]) {
      spdlog::error("Missing 'torque_sign' in elevate yaml");
      return ElevateConfig{};
    }
    if (!elevate_config["torque_sign"]["yaw1"]) {
      spdlog::error("Missing 'yaw1' in torque_sign config");
      return ElevateConfig{};
    }
    config.torque_sign.yaw1 =
        elevate_config["torque_sign"]["yaw1"].as<std::int32_t>();

    if (!elevate_config["torque_sign"]["yaw2"]) {
      spdlog::error("Missing 'yaw2' in torque_sign config");
      return ElevateConfig{};
    }
    config.torque_sign.yaw2 =
        elevate_config["torque_sign"]["yaw2"].as<std::int32_t>();

    if (!elevate_config["torque_sign"]["wrist_yaw"]) {
      spdlog::error("Missing 'wrist_yaw' in torque_sign config");
      return ElevateConfig{};
    }
    config.torque_sign.wrist_yaw =
        elevate_config["torque_sign"]["wrist_yaw"].as<std::int32_t>();
  } catch (const YAML::Exception& e) {
    spdlog::error("Failed to parse elevate config YAML: {}", e.what());
    return ElevateConfig{};
  } catch (const std::exception& e) {
    spdlog::error("Failed to load elevate config file: {}", e.what());
    return ElevateConfig{};
  }

  return config;
}

bool ValidateJointLimits(const JointLimitsConfig& config) {
  if (config.has_position_limits.empty()) {
    spdlog::error("Joint limits configuration is empty");
    return false;
  }
  return true;
}

bool ValidateButtonConfig(const ButtonConfig& config) {
  if (config.wrist_dial_min >= config.wrist_dial_max) {
    spdlog::error("wrist_dial_min ({}) must be less than wrist_dial_max ({})",
                  config.wrist_dial_min, config.wrist_dial_max);
    return false;
  }

  if ((config.wrist_dial_min < 0) || (config.wrist_dial_max <= 0) ||
      (config.analog_input_midpoint <= 0)) {
    spdlog::error(
        "wrist_dial_min ({}), wrist_dial_max ({}), and "
        "analog_input_midpoint ({}) must be greater than 0",
        config.wrist_dial_min, config.wrist_dial_max,
        config.analog_input_midpoint);
    return false;
  }

  if ((config.analog_input_midpoint < config.wrist_dial_min) ||
      (config.analog_input_midpoint > config.wrist_dial_max)) {
    spdlog::error(
        "analog_input_midpoint ({}) must be between wrist_dial_min ({}) and "
        "wrist_dial_max ({})",
        config.analog_input_midpoint, config.wrist_dial_min,
        config.wrist_dial_max);
    return false;
  }

  return true;
}

bool ValidateSpringSetpoints(const SpringSetpointsConfig& config) {
  if ((config.spring_setpoint_unloaded < 100) ||
      (config.spring_setpoint_unloaded > 1000)) {
    spdlog::error("Invalid spring_setpoint_unloaded in yaml config");
    return false;
  }

  if ((config.spring_setpoint_loaded < 100) ||
      (config.spring_setpoint_loaded > 3900)) {
    spdlog::error("Invalid spring_setpoint_loaded in yaml config");
    return false;
  }

  return true;
}

bool ValidateTorqueSignsConfig(const TorqueSignConfig& config) {
  if (((config.yaw1 != 1) && (config.yaw1 != -1)) ||
      ((config.yaw2 != 1) && (config.yaw2 != -1)) ||
      ((config.wrist_yaw != 1) && (config.wrist_yaw != -1))) {
    spdlog::error("Invalid torque_sign in yaml config: must be 1 or -1");
    return false;
  }

  return true;
}

}  // namespace elevated_control
