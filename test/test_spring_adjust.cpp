// Copyright (c) 2025 Elevate Robotics Inc

#include "elevated_control/constants.hpp"
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

TEST(CompleteSpringAdjustSession, ClearsSetpointAndSetsQuickStop) {
  std::optional<std::atomic<float>> setpoint;
  setpoint.emplace(123.0f);
  ControlLevel level = ControlLevel::kSpringAdjust;
  CompleteSpringAdjustSession(setpoint, level);
  EXPECT_FALSE(setpoint.has_value());
  EXPECT_EQ(level, ControlLevel::kQuickStop);
}

TEST(LoadNewtonsToSpringLipsTicks, ZeroNewtonsIsIntercept) {
  EXPECT_FLOAT_EQ(LoadNewtonsToSpringLipsTicks(0.0f), 161.1f);
}

TEST(LoadNewtonsToSpringLipsTicks, MatchesLinearModel) {
  // 3.7852 * 100 + 161.1
  EXPECT_FLOAT_EQ(LoadNewtonsToSpringLipsTicks(100.0f), 539.62f);
  EXPECT_FLOAT_EQ(LoadNewtonsToSpringLipsTicks(-10.0f), 123.248f);
}

}  // namespace
}  // namespace elevated_control
