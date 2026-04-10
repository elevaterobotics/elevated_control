// Copyright (c) 2026 Elevate Robotics Inc

#include <gtest/gtest.h>

#include "elevated_control/state_machine.hpp"
#include "elevated_control/types.hpp"
#include "elevated_control/types_arm.hpp"

namespace elevated_control {
namespace {

class DeadmanModeChangeStateTest : public ::testing::Test {
 protected:
  void SetUp() override { prev_deadman_pressed_ = false; }

  bool prev_deadman_pressed_;
  PreventModeChange prevent_mode_change_;
};

TEST_F(DeadmanModeChangeStateTest,
       HandGuidedModeDeadmanRisingEdgeBlocksModeChange) {
  UpdateDeadmanModeChangeState(ControlLevel::kHandGuided,
                               true /* deadman_pressed */,
                               prev_deadman_pressed_, prevent_mode_change_);

  EXPECT_TRUE(
      prevent_mode_change_.IsBlocked(ModeChangeBlockReason::kHandGuidedDeadman))
      << "Hand-guided blocker should be set when deadman is pressed in "
         "hand-guided mode";
  EXPECT_FALSE(prevent_mode_change_.AllowsNormalModeSwitch())
      << "Mode change should be blocked when deadman is pressed in "
         "hand-guided mode";
  EXPECT_TRUE(prev_deadman_pressed_)
      << "Previous deadman state should be updated to true";
}

TEST_F(DeadmanModeChangeStateTest, DeadmanFallingEdgeRestoresModeChange) {
  prev_deadman_pressed_ = true;
  prevent_mode_change_.SetBlocked(ModeChangeBlockReason::kHandGuidedDeadman);

  UpdateDeadmanModeChangeState(ControlLevel::kHandGuided,
                               false /* deadman_pressed */,
                               prev_deadman_pressed_, prevent_mode_change_);

  EXPECT_FALSE(
      prevent_mode_change_.IsBlocked(ModeChangeBlockReason::kHandGuidedDeadman))
      << "Hand-guided blocker should clear when deadman is released";
  EXPECT_TRUE(prevent_mode_change_.AllowsNormalModeSwitch())
      << "Mode change should be restored when deadman is released";
  EXPECT_FALSE(prev_deadman_pressed_)
      << "Previous deadman state should be updated to false";
}

TEST_F(DeadmanModeChangeStateTest, DeadmanHeldSteadyNoChange) {
  prev_deadman_pressed_ = true;
  prevent_mode_change_.SetBlocked(ModeChangeBlockReason::kHandGuidedDeadman);

  UpdateDeadmanModeChangeState(ControlLevel::kHandGuided,
                               true /* deadman_pressed */,
                               prev_deadman_pressed_, prevent_mode_change_);

  EXPECT_TRUE(
      prevent_mode_change_.IsBlocked(ModeChangeBlockReason::kHandGuidedDeadman))
      << "Mode change should remain blocked when deadman is held";
  EXPECT_TRUE(prev_deadman_pressed_)
      << "Previous deadman state should remain true";
}

TEST_F(DeadmanModeChangeStateTest, DeadmanReleasedSteadyNoChange) {
  prev_deadman_pressed_ = false;

  UpdateDeadmanModeChangeState(ControlLevel::kHandGuided,
                               false /* deadman_pressed */,
                               prev_deadman_pressed_, prevent_mode_change_);

  EXPECT_TRUE(prevent_mode_change_.AllowsNormalModeSwitch())
      << "Mode change should remain allowed when deadman stays released";
  EXPECT_FALSE(prev_deadman_pressed_)
      << "Previous deadman state should remain false";
}

TEST_F(DeadmanModeChangeStateTest,
       NonHandGuidedModeDeadmanPressDoesNotBlockModeChange) {
  UpdateDeadmanModeChangeState(ControlLevel::kPosition,
                               true /* deadman_pressed */,
                               prev_deadman_pressed_, prevent_mode_change_);

  EXPECT_TRUE(prevent_mode_change_.AllowsNormalModeSwitch())
      << "Mode change should not be blocked in non-hand-guided mode";
  EXPECT_FALSE(prev_deadman_pressed_)
      << "Previous deadman state should not be updated in non-hand-guided mode";
}

TEST_F(DeadmanModeChangeStateTest,
       DeadmanReleaseRestoresModeChangeRegardlessOfControlLevel) {
  prev_deadman_pressed_ = true;
  prevent_mode_change_.SetBlocked(ModeChangeBlockReason::kHandGuidedDeadman);

  UpdateDeadmanModeChangeState(ControlLevel::kPosition,
                               false /* deadman_pressed */,
                               prev_deadman_pressed_, prevent_mode_change_);

  EXPECT_TRUE(prevent_mode_change_.AllowsNormalModeSwitch())
      << "Mode change should be restored when deadman is released";
  EXPECT_FALSE(prev_deadman_pressed_)
      << "Previous deadman state should be updated to false";
}

TEST_F(DeadmanModeChangeStateTest,
       DeadmanReleaseDoesNotClearSpringAdjustBlocker) {
  prev_deadman_pressed_ = true;
  prevent_mode_change_.SetBlocked(ModeChangeBlockReason::kHandGuidedDeadman);
  prevent_mode_change_.SetBlocked(ModeChangeBlockReason::kSpringAdjust);

  UpdateDeadmanModeChangeState(ControlLevel::kPosition,
                               false /* deadman_pressed */,
                               prev_deadman_pressed_, prevent_mode_change_);

  EXPECT_FALSE(
      prevent_mode_change_.IsBlocked(ModeChangeBlockReason::kHandGuidedDeadman));
  EXPECT_TRUE(
      prevent_mode_change_.IsBlocked(ModeChangeBlockReason::kSpringAdjust))
      << "Deadman release should not clear unrelated blockers";
  EXPECT_FALSE(prevent_mode_change_.AllowsNormalModeSwitch())
      << "A remaining blocker should continue to block normal mode changes";
}

// -- PreventModeChange struct tests --

TEST(PreventModeChangeTest, DefaultAllowsModeSwitch) {
  PreventModeChange pmc;
  EXPECT_TRUE(pmc.AllowsNormalModeSwitch());
  EXPECT_FALSE(pmc.IsBlocked());
}

TEST(PreventModeChangeTest, SingleBlockerBlocks) {
  PreventModeChange pmc;
  pmc.SetBlocked(ModeChangeBlockReason::kEstop);
  EXPECT_TRUE(pmc.IsBlocked());
  EXPECT_TRUE(pmc.IsBlocked(ModeChangeBlockReason::kEstop));
  EXPECT_FALSE(pmc.IsBlocked(ModeChangeBlockReason::kSpringAdjust));
  EXPECT_FALSE(pmc.AllowsNormalModeSwitch());
}

TEST(PreventModeChangeTest, ClearRestoresAccess) {
  PreventModeChange pmc;
  pmc.SetBlocked(ModeChangeBlockReason::kEstop);
  pmc.ClearBlocked(ModeChangeBlockReason::kEstop);
  EXPECT_TRUE(pmc.AllowsNormalModeSwitch());
}

TEST(PreventModeChangeTest, MultipleBlockersRequireAllCleared) {
  PreventModeChange pmc;
  pmc.SetBlocked(ModeChangeBlockReason::kEstop);
  pmc.SetBlocked(ModeChangeBlockReason::kSpringAdjust);

  pmc.ClearBlocked(ModeChangeBlockReason::kEstop);
  EXPECT_FALSE(pmc.AllowsNormalModeSwitch())
      << "Clearing one blocker should not unblock when another is set";

  pmc.ClearBlocked(ModeChangeBlockReason::kSpringAdjust);
  EXPECT_TRUE(pmc.AllowsNormalModeSwitch());
}

TEST(PreventModeChangeTest, BlockingReasonsStringReportsActiveBlockers) {
  PreventModeChange pmc;
  EXPECT_EQ(pmc.BlockingReasonsString(), "none");

  pmc.SetBlocked(ModeChangeBlockReason::kHandGuidedDeadman);
  EXPECT_EQ(pmc.BlockingReasonsString(), "hand_guided");

  pmc.SetBlocked(ModeChangeBlockReason::kEstop);
  std::string reasons = pmc.BlockingReasonsString();
  EXPECT_NE(reasons.find("estop"), std::string::npos);
  EXPECT_NE(reasons.find("hand_guided"), std::string::npos);
}

}  // namespace
}  // namespace elevated_control
