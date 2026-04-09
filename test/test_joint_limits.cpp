#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>

#include "elevated_control/constants_arm.hpp"
#include "elevated_control/types_arm.hpp"
#include "elevated_control/joint_limits.hpp"
#include "elevated_control/unit_conversions.hpp"

using namespace elevated_control;

namespace {
inline constexpr float EPS = 1e-4f;
}

class JointLimitTest : public ::testing::Test {
 protected:
  void SetUp() override {
    joint_idx_ = 0;
    control_mode_ = ControlMode::kVelocity;
    has_position_limits_.fill(false);
    has_position_limits_[joint_idx_] = true;
    min_position_limits_.fill(-std::numeric_limits<float>::max());
    max_position_limits_.fill(std::numeric_limits<float>::max());
    min_position_limits_[joint_idx_] = -1.0f;
    max_position_limits_[joint_idx_] = 1.0f;
    max_brake_torques_ = kMaxJointLimitBrakeTorque;
    current_position_ = 0.0f;
    requested_command_ = 0.0f;

    std::memset(&in_somanet_, 0, sizeof(in_somanet_));
    std::memset(&out_somanet_, 0, sizeof(out_somanet_));
    wrap_value_.store(0.0f);
  }

  size_t joint_idx_;
  ControlMode control_mode_;
  JointBoolArray has_position_limits_{};
  JointFloatArray min_position_limits_{};
  JointFloatArray max_position_limits_{};
  std::array<float, kNumJoints> max_brake_torques_{};
  float current_position_;
  float requested_command_;
  InSomanet50t in_somanet_;
  OutSomanet50t out_somanet_;
  float mechanical_reduction_ = 1.0f;
  uint32_t encoder_resolution_ = 10000;
  VelocityFilter passthrough_filter_{0.0f};
  std::atomic<float> wrap_value_{0.0f};
};

TEST_F(JointLimitTest, HandGuidedControlModeReturnsValidResult) {
  control_mode_ = ControlMode::kTorque;
  current_position_ = 0.0f;
  requested_command_ = 0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  EXPECT_TRUE(result.has_value());
}

TEST_F(JointLimitTest, NoPositionLimitsReturnsOriginalCommand) {
  has_position_limits_[joint_idx_] = false;
  requested_command_ = 0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->first);
  EXPECT_FLOAT_EQ(result->second, requested_command_);
}

TEST_F(JointLimitTest, InfinitePositionLimitsReturnsOriginalCommand) {
  min_position_limits_[joint_idx_] = -std::numeric_limits<float>::infinity();
  max_position_limits_[joint_idx_] = std::numeric_limits<float>::infinity();
  requested_command_ = 0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->first);
  EXPECT_FLOAT_EQ(result->second, requested_command_);
}

TEST_F(JointLimitTest, NaNPositionLimitsReturnsOriginalCommand) {
  min_position_limits_[joint_idx_] =
      std::numeric_limits<float>::quiet_NaN();
  max_position_limits_[joint_idx_] =
      std::numeric_limits<float>::quiet_NaN();
  requested_command_ = 0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->first);
  EXPECT_FLOAT_EQ(result->second, requested_command_);
}

TEST_F(JointLimitTest, PositionInMiddleReturnsOriginalCommand) {
  current_position_ = 0.0f;
  requested_command_ = 0.3f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->first);
  EXPECT_FLOAT_EQ(result->second, requested_command_);
}

TEST_F(JointLimitTest, NearMinLimitPositiveVelocityReturnsOriginalCommand) {
  current_position_ =
      min_position_limits_[joint_idx_] + kPositionLimitBuffer + 0.01f;
  requested_command_ = 0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->second, requested_command_);
}

TEST_F(JointLimitTest, NearMinLimitNegativeVelocityScaledInBuffer) {
  current_position_ =
      min_position_limits_[joint_idx_] + kPositionLimitBuffer - 0.01f;
  requested_command_ = -0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  const float expected_scale =
      (current_position_ - min_position_limits_[joint_idx_]) /
      kPositionLimitBuffer;
  EXPECT_NEAR(result->second, requested_command_ * expected_scale, EPS);
}

TEST_F(JointLimitTest, AtMinLimitBufferNegativeVelocityScaledInBuffer) {
  current_position_ =
      min_position_limits_[joint_idx_] + kPositionLimitBuffer - 0.001f;
  requested_command_ = -0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  const float expected_scale =
      (current_position_ - min_position_limits_[joint_idx_]) /
      kPositionLimitBuffer;
  EXPECT_NEAR(result->second, requested_command_ * expected_scale, EPS);
}

TEST_F(JointLimitTest, NearMaxLimitNegativeVelocityReturnsOriginalCommand) {
  current_position_ =
      max_position_limits_[joint_idx_] - kPositionLimitBuffer - 0.01f;
  requested_command_ = -0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->second, requested_command_);
}

TEST_F(JointLimitTest, NearMaxLimitPositiveVelocityScaledInBuffer) {
  current_position_ =
      max_position_limits_[joint_idx_] - kPositionLimitBuffer + 0.01f;
  requested_command_ = 0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  const float expected_scale =
      (max_position_limits_[joint_idx_] - current_position_) /
      kPositionLimitBuffer;
  EXPECT_NEAR(result->second, requested_command_ * expected_scale, EPS);
}

TEST_F(JointLimitTest, AboveMaxLimitPositiveVelocityClampedToZero) {
  current_position_ = max_position_limits_[joint_idx_] + 0.1f;
  requested_command_ = 0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->second, 0.0f);
}

TEST_F(JointLimitTest, BelowMinLimitNegativeVelocityClampedToZero) {
  current_position_ = min_position_limits_[joint_idx_] - 0.1f;
  requested_command_ = -0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->second, 0.0f);
}

TEST_F(JointLimitTest, ZeroVelocityCommandAlwaysUnchanged) {
  requested_command_ = 0.0f;

  current_position_ =
      min_position_limits_[joint_idx_] + kPositionLimitBuffer - 0.01f;
  auto result_min =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  current_position_ =
      max_position_limits_[joint_idx_] - kPositionLimitBuffer + 0.01f;
  auto result_max =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result_min.has_value());
  ASSERT_TRUE(result_max.has_value());
  EXPECT_FLOAT_EQ(result_min->second, 0.0f);
  EXPECT_FLOAT_EQ(result_max->second, 0.0f);
}

TEST_F(JointLimitTest, PositionAtMinimumLimit) {
  current_position_ = min_position_limits_[joint_idx_];
  requested_command_ = -0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->second, 0.0f);
}

TEST_F(JointLimitTest, PositionAtMaximumLimit) {
  current_position_ = max_position_limits_[joint_idx_];
  requested_command_ = 0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->second, 0.0f);
}

// -- Position control mode tests --

TEST_F(JointLimitTest, PositionControlModeMiddleRangeReturnsOriginalCommand) {
  control_mode_ = ControlMode::kPosition;
  current_position_ = 0.0f;
  requested_command_ = 0.3f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->second, requested_command_);
}

TEST_F(JointLimitTest, PositionControlModeNearMinLimitClampedToBuffer) {
  control_mode_ = ControlMode::kPosition;
  current_position_ =
      min_position_limits_[joint_idx_] + kPositionLimitBuffer - 0.01f;
  requested_command_ = -0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->second,
                   min_position_limits_[joint_idx_] + kPositionLimitBuffer);
}

TEST_F(JointLimitTest, PositionControlModeNearMaxLimitClampedToBuffer) {
  control_mode_ = ControlMode::kPosition;
  current_position_ =
      max_position_limits_[joint_idx_] - kPositionLimitBuffer + 0.01f;
  requested_command_ = 0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->second,
                   max_position_limits_[joint_idx_] - kPositionLimitBuffer);
}

// -- Torque control mode tests --

TEST_F(JointLimitTest, TorqueControlModeNoPositionLimitsReturnsOriginalCommand) {
  control_mode_ = ControlMode::kTorque;
  has_position_limits_[joint_idx_] = false;
  requested_command_ = 0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->second, requested_command_);
}

TEST_F(JointLimitTest,
       TorqueControlModeBelowMinLimitBufferAddsMaxBrakeTorque) {
  control_mode_ = ControlMode::kTorque;
  current_position_ =
      min_position_limits_[joint_idx_] + kPositionLimitBuffer - 0.01f;
  requested_command_ = 0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->second,
                   requested_command_ + kMaxJointLimitBrakeTorque[joint_idx_]);
}

TEST_F(JointLimitTest,
       TorqueControlModeAboveMaxLimitBufferSubtractsMaxBrakeTorque) {
  control_mode_ = ControlMode::kTorque;
  current_position_ =
      max_position_limits_[joint_idx_] - kPositionLimitBuffer + 0.01f;
  requested_command_ = 0.5f;

  auto result =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->second,
                   requested_command_ - kMaxJointLimitBrakeTorque[joint_idx_]);
}

TEST_F(JointLimitTest, TorqueControlModeZeroCommandModifiedByBrakeTorque) {
  control_mode_ = ControlMode::kTorque;
  requested_command_ = 0.0f;

  current_position_ =
      min_position_limits_[joint_idx_] + kPositionLimitBuffer - 0.01f;
  auto result_min =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  current_position_ =
      max_position_limits_[joint_idx_] - kPositionLimitBuffer + 0.01f;
  auto result_max =
      JointLimitCmdClamp(joint_idx_, control_mode_, current_position_,
                         has_position_limits_, min_position_limits_,
                         max_position_limits_, max_brake_torques_,
                         requested_command_);

  ASSERT_TRUE(result_min.has_value());
  ASSERT_TRUE(result_max.has_value());
  EXPECT_NEAR(result_min->second, kMaxJointLimitBrakeTorque[joint_idx_], EPS);
  EXPECT_NEAR(result_max->second, -kMaxJointLimitBrakeTorque[joint_idx_], EPS);
}

// -- SetVelocityWithLimits tests --

TEST_F(JointLimitTest, SetVelocityWithLimits_NormalOperation) {
  in_somanet_.PositionValue = 0;
  requested_command_ = 1.0f;
  const int32_t si_velocity_unit = 0;
  const int32_t expected = OutputShaftRadPerSToVelocityValue(
      requested_command_, si_velocity_unit);

  SetVelocityWithLimits(joint_idx_, &in_somanet_, has_position_limits_,
                        min_position_limits_, max_position_limits_,
                        max_brake_torques_,
                        mechanical_reduction_, encoder_resolution_,
                        requested_command_, si_velocity_unit,
                        wrap_value_,
                        passthrough_filter_, &out_somanet_);

  EXPECT_EQ(out_somanet_.TargetVelocity, expected);
  EXPECT_EQ(out_somanet_.OpMode, kCyclicVelocityMode);
  EXPECT_EQ(out_somanet_.VelocityOffset, 0);
}

TEST_F(JointLimitTest, SetVelocityWithLimits_ScalesVelocityInBuffer) {
  float desired_pos = -0.96f;
  int32_t ticks = static_cast<int32_t>(
      desired_pos * mechanical_reduction_ * encoder_resolution_ /
      (2.0f * static_cast<float>(M_PI)));

  in_somanet_.PositionValue = ticks;
  requested_command_ = -0.5f;

  // Compute startup wrap for this joint
  ComputeStartupWrapOffset(ticks, mechanical_reduction_, encoder_resolution_,
                           wrap_value_);

  const int32_t si_velocity_unit = 0;
  const float actual_position = InputTicksToOutputShaftRad(
      in_somanet_.PositionValue, mechanical_reduction_, encoder_resolution_,
      wrap_value_);
  const float expected_velocity =
      requested_command_ *
      ((actual_position - min_position_limits_[joint_idx_]) /
       kPositionLimitBuffer);
  SetVelocityWithLimits(joint_idx_, &in_somanet_, has_position_limits_,
                        min_position_limits_, max_position_limits_,
                        max_brake_torques_,
                        mechanical_reduction_, encoder_resolution_,
                        requested_command_, si_velocity_unit,
                        wrap_value_,
                        passthrough_filter_, &out_somanet_);

  EXPECT_EQ(out_somanet_.TargetVelocity,
            OutputShaftRadPerSToVelocityValue(
                expected_velocity, si_velocity_unit));
  EXPECT_EQ(out_somanet_.OpMode, kCyclicVelocityMode);
  EXPECT_EQ(out_somanet_.VelocityOffset, 0);
}

TEST_F(JointLimitTest, SetVelocityWithLimits_NormalOperation_MilliRpmUnit) {
  const int32_t si_velocity_unit =
      static_cast<int32_t>(0xFDB44700u);
  in_somanet_.PositionValue = 0;
  requested_command_ = 1.0f;

  SetVelocityWithLimits(joint_idx_, &in_somanet_, has_position_limits_,
                        min_position_limits_, max_position_limits_,
                        max_brake_torques_,
                        mechanical_reduction_, encoder_resolution_,
                        requested_command_, si_velocity_unit,
                        wrap_value_,
                        passthrough_filter_, &out_somanet_);

  int32_t expected = OutputShaftRadPerSToVelocityValue(
      1.0f, si_velocity_unit);
  EXPECT_NEAR(out_somanet_.TargetVelocity, expected, 1);
  EXPECT_EQ(out_somanet_.OpMode, kCyclicVelocityMode);
  EXPECT_EQ(out_somanet_.VelocityOffset, 0);
}
