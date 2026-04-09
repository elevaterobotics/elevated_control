// Copyright (c) 2025 Elevate Robotics Inc

#include "elevated_control/constants_arm.hpp"
#include "elevated_control/spring_adjust.hpp"
#include "elevated_control/types.hpp"

#include <gtest/gtest.h>

#include <atomic>

namespace elevated_control {
namespace {

SpringAdjustState MakeFreshState() {
  SpringAdjustState state{};
  state.time_prev = std::chrono::steady_clock::now();
  state.error_prev = std::nullopt;
  return state;
}

TEST(SpringAdjustByLIPS, PositiveErrorClampsToMaxTorque) {
  SpringAdjustState state = MakeFreshState();
  bool allow = false;
  float t = SpringAdjustByLIPS(10'000.0f, 0, allow, state);
  EXPECT_EQ(t, kSpringAdjustMaxTorque);
  EXPECT_FALSE(allow);
}

TEST(SpringAdjustByLIPS, NegativeErrorClampsToMinMagnitudeTorque) {
  SpringAdjustState state = MakeFreshState();
  bool allow = false;
  float t = SpringAdjustByLIPS(0.0f, 10'000, allow, state);
  EXPECT_EQ(t, -kSpringAdjustMaxTorque);
  EXPECT_FALSE(allow);
}

TEST(SpringAdjustByLIPS, WithinThresholdZerosTorqueAndSetsAllowWhenFalse) {
  SpringAdjustState state = MakeFreshState();
  bool allow = false;
  float t = SpringAdjustByLIPS(1000.0f, 1000, allow, state);
  EXPECT_FLOAT_EQ(t, 0.0f);
  EXPECT_TRUE(allow);
}

TEST(SpringAdjustByLIPS, WithinThresholdDoesNotClearAllowWhenAlreadyTrue) {
  SpringAdjustState state = MakeFreshState();
  bool allow = true;
  float t = SpringAdjustByLIPS(1000.0f, 1000, allow, state);
  EXPECT_FLOAT_EQ(t, 0.0f);
  EXPECT_TRUE(allow);
}

TEST(SpringAdjustByLIPS, ErrorJustBelowThresholdCompletes) {
  SpringAdjustState state = MakeFreshState();
  bool allow = false;
  // |1000 - 951| = 49 < 50
  float t = SpringAdjustByLIPS(1000.0f, 951, allow, state);
  EXPECT_FLOAT_EQ(t, 0.0f);
  EXPECT_TRUE(allow);
}

TEST(SpringAdjustByLIPS, ErrorAtThresholdDoesNotComplete) {
  SpringAdjustState state = MakeFreshState();
  bool allow = false;
  // |1000 - 950| = 50, not < 50
  float t = SpringAdjustByLIPS(1000.0f, 950, allow, state);
  EXPECT_NE(t, 0.0f);
  EXPECT_FALSE(allow);
}

TEST(SpringAdjustSetpointStorage, StoresAndOverwritesTarget) {
  std::atomic<float> setpoint_ticks{0.0f};
  std::atomic<bool> has_setpoint{false};

  EXPECT_FALSE(
      LoadSpringAdjustSetpoint(setpoint_ticks, has_setpoint).has_value());

  StoreSpringAdjustSetpoint(setpoint_ticks, has_setpoint, 123.0f);
  auto first_setpoint = LoadSpringAdjustSetpoint(setpoint_ticks, has_setpoint);
  ASSERT_TRUE(first_setpoint.has_value());
  EXPECT_FLOAT_EQ(*first_setpoint, 123.0f);

  StoreSpringAdjustSetpoint(setpoint_ticks, has_setpoint, 456.0f);
  auto second_setpoint = LoadSpringAdjustSetpoint(setpoint_ticks, has_setpoint);
  ASSERT_TRUE(second_setpoint.has_value());
  EXPECT_FLOAT_EQ(*second_setpoint, 456.0f);
}

TEST(CompleteSpringAdjustSession, ClearsSetpointAndSetsQuickStop) {
  std::atomic<float> setpoint_ticks{123.0f};
  std::atomic<bool> has_setpoint{true};
  ControlMode mode = ControlMode::kTorque;
  CompleteSpringAdjustSession(has_setpoint, mode);
  EXPECT_FALSE(LoadSpringAdjustSetpoint(setpoint_ticks, has_setpoint).has_value());
  EXPECT_EQ(mode, ControlMode::kQuickStop);
}

TEST(SpringAdjustSetpointStorage, CanStoreNewTargetAfterCompletion) {
  std::atomic<float> setpoint_ticks{123.0f};
  std::atomic<bool> has_setpoint{true};
  ControlMode mode = ControlMode::kTorque;

  CompleteSpringAdjustSession(has_setpoint, mode);
  StoreSpringAdjustSetpoint(setpoint_ticks, has_setpoint, 789.0f);

  auto new_setpoint = LoadSpringAdjustSetpoint(setpoint_ticks, has_setpoint);
  ASSERT_TRUE(new_setpoint.has_value());
  EXPECT_FLOAT_EQ(*new_setpoint, 789.0f);
  EXPECT_EQ(mode, ControlMode::kQuickStop);
}

TEST(ConvertNewtonsToSpringLipsTicks, ZeroNewtonsIsIntercept) {
  EXPECT_FLOAT_EQ(ConvertNewtonsToSpringLipsTicks(0.0f), 161.1f);
}

TEST(ConvertNewtonsToSpringLipsTicks, MatchesLinearModel) {
  EXPECT_FLOAT_EQ(ConvertNewtonsToSpringLipsTicks(100.0f), 539.62f);
  EXPECT_FLOAT_EQ(ConvertNewtonsToSpringLipsTicks(-10.0f), 123.248f);
}

}  // namespace
}  // namespace elevated_control
