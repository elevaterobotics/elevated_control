#include <gtest/gtest.h>

#include <array>
#include <memory>

#include "elevated_control/state_machine.hpp"
#include "elevated_control/types.hpp"

using namespace elevated_control;

namespace {

class StateMachineHelperTest : public ::testing::Test {
 protected:
  void SetUp() override {
    control_level_.fill(ControlLevel::kHandGuided);
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

  JointControlLevelArray control_level_{};
  JointArray<OutSomanet50t*> out_somanet_ptrs_{};
  std::array<std::unique_ptr<OutSomanet50t>, kNumJoints> out_somanet_objects_{};
};

TEST_F(StateMachineHelperTest, HandleShutdownKeepsJointDisabledWhileLatched) {
  constexpr std::size_t kJointIdx = kWristPitchIdx;

  HandleShutdown(out_somanet_ptrs_, kJointIdx, control_level_,
                 false /* mode_switch_in_progress */,
                 true /* hold_in_shutdown */);

  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->TargetVelocity, 0);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->VelocityOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->TargetTorque, 0);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->TorqueOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->OpMode, kCyclicVelocityMode);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->Controlword, 0);
}

TEST_F(StateMachineHelperTest, HandleSwitchOnReturnsLatchedJointToShutdown) {
  constexpr std::size_t kJointIdx = kWristPitchIdx;

  HandleSwitchOn(out_somanet_ptrs_, kJointIdx, true /* hold_in_shutdown */);

  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->TargetVelocity, 0);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->VelocityOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->TargetTorque, 0);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->TorqueOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->OpMode, kCyclicVelocityMode);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->Controlword, 0);
}

TEST_F(StateMachineHelperTest,
       HandleEnableOperationReturnsLatchedJointToShutdown) {
  constexpr std::size_t kJointIdx = kWristPitchIdx;

  HandleEnableOperation(out_somanet_ptrs_, kJointIdx,
                        false /* mode_switch_in_progress */,
                        ControlLevel::kHandGuided,
                        true /* deadman_pressed */,
                        true /* hold_in_shutdown */);

  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->TargetVelocity, 0);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->VelocityOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->TargetTorque, 0);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->TorqueOffset, 0);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->OpMode, kCyclicVelocityMode);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->Controlword, 0);
}

TEST_F(StateMachineHelperTest, ClearingHoldRestoresNormalEnableProgression) {
  constexpr std::size_t kJointIdx = kWristPitchIdx;

  HandleShutdown(out_somanet_ptrs_, kJointIdx, control_level_,
                 false /* mode_switch_in_progress */,
                 true /* hold_in_shutdown */);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->Controlword, 0);

  HandleShutdown(out_somanet_ptrs_, kJointIdx, control_level_,
                 false /* mode_switch_in_progress */,
                 false /* hold_in_shutdown */);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->Controlword, 0b00000110);

  HandleSwitchOn(out_somanet_ptrs_, kJointIdx, false /* hold_in_shutdown */);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->Controlword, 0b00000111);

  HandleEnableOperation(out_somanet_ptrs_, kJointIdx,
                        false /* mode_switch_in_progress */,
                        ControlLevel::kHandGuided,
                        true /* deadman_pressed */,
                        false /* hold_in_shutdown */);
  EXPECT_EQ(out_somanet_ptrs_[kJointIdx]->Controlword, kNormalOpBrakesOff);
}

}  // namespace
