// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <span>
#include <utility>
#include <spdlog/spdlog.h>

#include "elevated_control/constants.hpp"
#include "elevated_control/somanet_pdo.hpp"
#include "elevated_control/types.hpp"
#include "elevated_control/unit_conversions.hpp"
#include "elevated_control/velocity_filter.hpp"

namespace elevated_control {

inline std::optional<std::pair<bool, float>> JointLimitCmdClamp(
    const std::size_t joint_idx, const ControlMode control_mode,
    const float current_position, std::span<const bool> has_position_limits,
    std::span<const float> min_position_limits,
    std::span<const float> max_position_limits,
    std::span<const float> max_brake_torques,
    const float requested_command) {
  if (!has_position_limits[joint_idx] ||
      !std::isfinite(min_position_limits[joint_idx]) ||
      !std::isfinite(max_position_limits[joint_idx])) {
    return std::make_pair(false, requested_command);
  }

  float output_command = requested_command;
  bool near_joint_limit = false;

  if (control_mode == ControlMode::kVelocity) {
    if (current_position <
        min_position_limits[joint_idx] + kPositionLimitBuffer) {
      if (requested_command > 0.0f) {
        output_command = requested_command;
      } else if (current_position < min_position_limits[joint_idx]) {
        output_command = 0.0f;
      } else {
        const float scale =
            (current_position - min_position_limits[joint_idx]) /
            kPositionLimitBuffer;
        output_command = requested_command * std::clamp(scale, 0.0f, 1.0f);
      }
      near_joint_limit = true;
    } else if (current_position >
               max_position_limits[joint_idx] - kPositionLimitBuffer) {
      if (requested_command < 0.0f) {
        output_command = requested_command;
      } else if (current_position > max_position_limits[joint_idx]) {
        output_command = 0.0f;
      } else {
        const float scale =
            (max_position_limits[joint_idx] - current_position) /
            kPositionLimitBuffer;
        output_command = requested_command * std::clamp(scale, 0.0f, 1.0f);
      }
      near_joint_limit = true;
    }
  } else if (control_mode == ControlMode::kPosition) {
    if (current_position <
        min_position_limits[joint_idx] + kPositionLimitBuffer) {
      output_command = min_position_limits[joint_idx] + kPositionLimitBuffer;
      near_joint_limit = true;
    } else if (current_position >
               max_position_limits[joint_idx] - kPositionLimitBuffer) {
      output_command = max_position_limits[joint_idx] - kPositionLimitBuffer;
      near_joint_limit = true;
    }
  } else if (control_mode == ControlMode::kTorque) {
    float distance_to_upper =
        max_position_limits[joint_idx] - kPositionLimitBuffer -
        current_position;
    float distance_to_lower =
        current_position -
        (min_position_limits[joint_idx] + kPositionLimitBuffer);

    if (current_position <
        min_position_limits[joint_idx] + kPositionLimitBuffer) {
      output_command += max_brake_torques[joint_idx];
      near_joint_limit = true;
    } else if (current_position >
               max_position_limits[joint_idx] - kPositionLimitBuffer) {
      output_command -= max_brake_torques[joint_idx];
      near_joint_limit = true;
    } else if (distance_to_upper < distance_to_lower) {
      if (distance_to_upper < kJointLimitTorqueBrakeDistance) {
        float torque_fraction =
            (kJointLimitTorqueBrakeDistance - distance_to_upper) /
            kJointLimitTorqueBrakeDistance;
        float brake_torque =
            torque_fraction * max_brake_torques[joint_idx];
        output_command -= brake_torque;
        near_joint_limit = true;
      }
    } else if (distance_to_lower < distance_to_upper) {
      if (distance_to_lower < kJointLimitTorqueBrakeDistance) {
        float torque_fraction =
            (kJointLimitTorqueBrakeDistance - distance_to_lower) /
            kJointLimitTorqueBrakeDistance;
        float brake_torque =
            torque_fraction * max_brake_torques[joint_idx];
        output_command += brake_torque;
        near_joint_limit = true;
      }
    }
  } else {
    spdlog::error("JointLimitCmdClamp: invalid control mode");
    return std::nullopt;
  }

  return std::make_pair(near_joint_limit, output_command);
}

// `requested_velocity` is output-shaft rad/s before conversion to drive units.
inline void SetVelocityWithLimits(
    const std::size_t joint_idx, const InSomanet50t* in_somanet,
    std::span<const bool> has_position_limits,
    std::span<const float> min_position_limits,
    std::span<const float> max_position_limits,
    std::span<const float> max_brake_torques,
    const float mechanical_reduction, const std::uint32_t encoder_resolution,
    const float requested_velocity, const std::int32_t si_velocity_unit,
    const std::atomic<float>& wrap_value,
    VelocityFilter& velocity_filter,
    OutSomanet50t* out_somanet) {
  float output_velocity = requested_velocity;
  if (has_position_limits[joint_idx] &&
      std::isfinite(min_position_limits[joint_idx]) &&
      std::isfinite(max_position_limits[joint_idx])) {
    float current_position = InputTicksToOutputShaftRad(
        in_somanet->PositionValue, mechanical_reduction, encoder_resolution,
        wrap_value);
    auto result = JointLimitCmdClamp(
        joint_idx, ControlMode::kVelocity, current_position,
        has_position_limits, min_position_limits, max_position_limits,
        max_brake_torques, output_velocity);

    if (result.has_value()) {
      output_velocity = result.value().second;
    }
  }
  output_velocity = velocity_filter.Filter(output_velocity);
  out_somanet->TargetVelocity = OutputShaftRadPerSToVelocityValue(
      output_velocity, si_velocity_unit, mechanical_reduction,
      encoder_resolution);
  out_somanet->OpMode = kCyclicVelocityMode;
  out_somanet->VelocityOffset = 0;
}

}  // namespace elevated_control
