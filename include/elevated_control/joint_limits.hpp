// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include "elevated_control/constants.hpp"
#include "elevated_control/somanet_pdo.hpp"
#include "elevated_control/types.hpp"
#include "elevated_control/unit_conversions.hpp"

namespace elevated_control {

inline std::optional<std::pair<bool, float>> JointLimitCmdClamp(
    const std::size_t joint_idx, const ControlLevel control_level,
    const float current_position, const std::vector<bool>& has_position_limits,
    const std::vector<float>& min_position_limits,
    const std::vector<float>& max_position_limits,
    const float requested_command) {
  if (!has_position_limits[joint_idx] ||
      !std::isfinite(min_position_limits[joint_idx]) ||
      !std::isfinite(max_position_limits[joint_idx])) {
    return std::make_pair(false, requested_command);
  }

  float output_command = requested_command;
  bool near_joint_limit = false;

  if (control_level == ControlLevel::kVelocity) {
    if (current_position <
        min_position_limits[joint_idx] + kPositionLimitBuffer) {
      output_command = std::max(requested_command, 0.0f);
      near_joint_limit = true;
    } else if (current_position >
               max_position_limits[joint_idx] - kPositionLimitBuffer) {
      output_command = std::min(requested_command, 0.0f);
      near_joint_limit = true;
    }
  } else if (control_level == ControlLevel::kPosition) {
    if (current_position <
        min_position_limits[joint_idx] + kPositionLimitBuffer) {
      output_command = min_position_limits[joint_idx] + kPositionLimitBuffer;
      near_joint_limit = true;
    } else if (current_position >
               max_position_limits[joint_idx] - kPositionLimitBuffer) {
      output_command = max_position_limits[joint_idx] - kPositionLimitBuffer;
      near_joint_limit = true;
    }
  } else if (control_level == ControlLevel::kTorque ||
             control_level == ControlLevel::kHandGuided) {
    float distance_to_upper =
        max_position_limits[joint_idx] - kPositionLimitBuffer -
        current_position;
    float distance_to_lower =
        current_position -
        (min_position_limits[joint_idx] + kPositionLimitBuffer);

    if (current_position <
        min_position_limits[joint_idx] + kPositionLimitBuffer) {
      output_command += kMaxJointLimitBrakeTorque[joint_idx];
      near_joint_limit = true;
    } else if (current_position >
               max_position_limits[joint_idx] - kPositionLimitBuffer) {
      output_command -= kMaxJointLimitBrakeTorque[joint_idx];
      near_joint_limit = true;
    } else if (distance_to_upper < distance_to_lower) {
      if (distance_to_upper < kJointLimitTorqueBrakeDistance) {
        float torque_fraction =
            (kJointLimitTorqueBrakeDistance - distance_to_upper) /
            kJointLimitTorqueBrakeDistance;
        float brake_torque =
            torque_fraction * kMaxJointLimitBrakeTorque[joint_idx];
        output_command -= brake_torque;
        near_joint_limit = true;
      }
    } else if (distance_to_lower < distance_to_upper) {
      if (distance_to_lower < kJointLimitTorqueBrakeDistance) {
        float torque_fraction =
            (kJointLimitTorqueBrakeDistance - distance_to_lower) /
            kJointLimitTorqueBrakeDistance;
        float brake_torque =
            torque_fraction * kMaxJointLimitBrakeTorque[joint_idx];
        output_command += brake_torque;
        near_joint_limit = true;
      }
    }
  } else {
    spdlog::error("JointLimitCmdClamp: invalid control level");
    return std::nullopt;
  }

  return std::make_pair(near_joint_limit, output_command);
}

inline void SetVelocityWithLimits(
    const std::size_t joint_idx, const InSomanet50t* in_somanet,
    const std::vector<bool>& has_position_limits,
    const std::vector<float>& min_position_limits,
    const std::vector<float>& max_position_limits,
    const float mechanical_reduction, const std::uint32_t encoder_resolution,
    const float requested_velocity, OutSomanet50t* out_somanet) {
  float output_velocity = requested_velocity;
  if (has_position_limits[joint_idx] &&
      std::isfinite(min_position_limits[joint_idx]) &&
      std::isfinite(max_position_limits[joint_idx])) {
    float current_position = InputTicksToOutputShaftRad(
        in_somanet->PositionValue, mechanical_reduction, encoder_resolution,
        joint_idx);
    auto result = JointLimitCmdClamp(
        joint_idx, ControlLevel::kVelocity, current_position,
        has_position_limits, min_position_limits, max_position_limits,
        output_velocity);

    if (result.has_value()) {
      output_velocity = result.value().second;
    }
  }
  out_somanet->TargetVelocity = static_cast<std::int32_t>(output_velocity);
  out_somanet->OpMode = kCyclicVelocityMode;
  out_somanet->VelocityOffset = 0;
}

}  // namespace elevated_control
