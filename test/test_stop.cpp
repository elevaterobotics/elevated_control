#include <gtest/gtest.h>
#include <array>
#include <memory>

#include "elevated_control/constants_arm.hpp"
#include "elevated_control/types_arm.hpp"
#include "elevated_control/state_machine.hpp"

using namespace elevated_control;

namespace {

class StopFunctionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      auto out_somanet = std::make_unique<OutSomanet50t>();
      out_somanet->OpMode = kOriginalOpMode;
      out_somanet->TargetVelocity = 100;
      out_somanet->VelocityOffset = 50;
      out_somanet->TargetTorque = 200;
      out_somanet->TorqueOffset = 75;
      out_somanet->Controlword = 0x1234;

      out_somanet_ptrs_[i] = out_somanet.get();
      out_somanet_objects_[i] = std::move(out_somanet);
    }
  }

  static constexpr std::int8_t kOriginalOpMode = 1;

  JointArray<OutSomanet50t*> out_somanet_ptrs_{};
  std::array<std::unique_ptr<OutSomanet50t>, kNumJoints> out_somanet_objects_{};
};

TEST_F(StopFunctionTest, GeneralJointWithoutBrake) {
  const int joint_idx = 0;
  const bool apply_brake = false;

  Stop(out_somanet_ptrs_[joint_idx], apply_brake);

  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->TargetVelocity, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->VelocityOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->TargetTorque, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->TorqueOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->OpMode, kCyclicVelocityMode);
  // Without brake, Controlword is not touched
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->Controlword, 0x1234);
}

TEST_F(StopFunctionTest, GeneralJointWithBrake) {
  const int joint_idx = 1;
  const bool apply_brake = true;

  Stop(out_somanet_ptrs_[joint_idx], apply_brake);

  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->TargetVelocity, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->VelocityOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->TargetTorque, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->TorqueOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->OpMode, kCyclicVelocityMode);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->Controlword, 0);
}

TEST_F(StopFunctionTest, OtherJointsUnaffected) {
  const int target_joint_idx = 2;
  const bool apply_brake = false;

  std::array<std::int8_t, kNumJoints> original_op_modes{};
  std::array<std::int32_t, kNumJoints> original_target_velocities{};
  std::array<std::int32_t, kNumJoints> original_velocity_offsets{};
  std::array<std::int16_t, kNumJoints> original_target_torques{};
  std::array<std::int16_t, kNumJoints> original_torque_offsets{};
  std::array<std::uint16_t, kNumJoints> original_controlwords{};

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    original_op_modes[i] = out_somanet_ptrs_[i]->OpMode;
    original_target_velocities[i] = out_somanet_ptrs_[i]->TargetVelocity;
    original_velocity_offsets[i] = out_somanet_ptrs_[i]->VelocityOffset;
    original_target_torques[i] = out_somanet_ptrs_[i]->TargetTorque;
    original_torque_offsets[i] = out_somanet_ptrs_[i]->TorqueOffset;
    original_controlwords[i] = out_somanet_ptrs_[i]->Controlword;
  }

  Stop(out_somanet_ptrs_[target_joint_idx], apply_brake);

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i != static_cast<std::size_t>(target_joint_idx)) {
      EXPECT_EQ(out_somanet_ptrs_[i]->OpMode, original_op_modes[i]);
      EXPECT_EQ(out_somanet_ptrs_[i]->TargetVelocity,
                original_target_velocities[i]);
      EXPECT_EQ(out_somanet_ptrs_[i]->VelocityOffset,
                original_velocity_offsets[i]);
      EXPECT_EQ(out_somanet_ptrs_[i]->TargetTorque,
                original_target_torques[i]);
      EXPECT_EQ(out_somanet_ptrs_[i]->TorqueOffset,
                original_torque_offsets[i]);
      EXPECT_EQ(out_somanet_ptrs_[i]->Controlword,
                original_controlwords[i]);
    }
  }
}

TEST_F(StopFunctionTest, MultipleCallsOnSameJoint) {
  const int joint_idx = 3;

  Stop(out_somanet_ptrs_[joint_idx], false);

  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->TargetVelocity, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->VelocityOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->TargetTorque, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->TorqueOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->OpMode, kCyclicVelocityMode);

  out_somanet_ptrs_[joint_idx]->TargetVelocity = 100;
  out_somanet_ptrs_[joint_idx]->VelocityOffset = 50;
  out_somanet_ptrs_[joint_idx]->TargetTorque = 150;
  out_somanet_ptrs_[joint_idx]->TorqueOffset = 75;
  out_somanet_ptrs_[joint_idx]->Controlword = 0x1234;

  Stop(out_somanet_ptrs_[joint_idx], true);

  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->TargetVelocity, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->VelocityOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->TargetTorque, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->TorqueOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->OpMode, kCyclicVelocityMode);
  EXPECT_EQ(out_somanet_ptrs_[joint_idx]->Controlword, 0);
}

}  // namespace
