// Copyright (c) 2025 Elevate Robotics Inc

#include <gtest/gtest.h>

#include "elevated_control/constants.hpp"
#include "elevated_control/wrist_pitch_dial.hpp"

namespace {

constexpr float kEps = 1e-6f;

float ExpectedVelocityPositiveRamp(float normalized_dial) {
  const float v_min = -elevated_control::kMinWristPitchDialVelocity;
  const float v_max = elevated_control::kMaxWristPitchVelocity;
  const float slope =
      (v_max - v_min) / (1.0f - elevated_control::kWristPitchDeadband);
  return slope * (normalized_dial - elevated_control::kWristPitchDeadband) +
         v_min;
}

float ExpectedVelocityNegativeRamp(float normalized_dial) {
  const float v_min = -elevated_control::kMinWristPitchDialVelocity;
  const float v_max = elevated_control::kMaxWristPitchVelocity;
  const float slope =
      (v_max - v_min) / (1.0f - elevated_control::kWristPitchDeadband);
  return slope *
             (normalized_dial - (-elevated_control::kWristPitchDeadband)) -
         v_min;
}

}  // namespace

TEST(WristPitchDialVelocity, ZeroAtNeutral) {
  EXPECT_FLOAT_EQ(elevated_control::WristPitchDialNormalizedToVelocity(0.0f),
                  0.0f);
}

TEST(WristPitchDialVelocity, RampEndpointsAtDeadband) {
  using elevated_control::kMaxWristPitchVelocity;
  using elevated_control::kMinWristPitchDialVelocity;
  using elevated_control::kWristPitchDeadband;
  using elevated_control::WristPitchDialNormalizedToVelocity;

  EXPECT_NEAR(WristPitchDialNormalizedToVelocity(kWristPitchDeadband),
              -kMinWristPitchDialVelocity, kEps);
  EXPECT_NEAR(WristPitchDialNormalizedToVelocity(-kWristPitchDeadband),
              kMinWristPitchDialVelocity, kEps);
  EXPECT_NEAR(WristPitchDialNormalizedToVelocity(1.0f), kMaxWristPitchVelocity,
              kEps);
  EXPECT_NEAR(WristPitchDialNormalizedToVelocity(-1.0f), -kMaxWristPitchVelocity,
              kEps);
}

TEST(WristPitchDialVelocity, SaturatesOutsideUnitRange) {
  using elevated_control::kMaxWristPitchVelocity;
  using elevated_control::WristPitchDialNormalizedToVelocity;

  EXPECT_FLOAT_EQ(WristPitchDialNormalizedToVelocity(1.5f),
                  kMaxWristPitchVelocity);
  EXPECT_FLOAT_EQ(WristPitchDialNormalizedToVelocity(-1.5f),
                  -kMaxWristPitchVelocity);
}

TEST(WristPitchDialVelocity, InteriorPointsMatchLinearRamp) {
  using elevated_control::WristPitchDialNormalizedToVelocity;

  constexpr float kMidPos = 0.5f;
  constexpr float kMidNeg = -0.5f;
  EXPECT_NEAR(WristPitchDialNormalizedToVelocity(kMidPos),
              ExpectedVelocityPositiveRamp(kMidPos), kEps);
  EXPECT_NEAR(WristPitchDialNormalizedToVelocity(kMidNeg),
              ExpectedVelocityNegativeRamp(kMidNeg), kEps);
}

TEST(WristPitchDialVelocity, StrictDeadbandInteriorIsZero) {
  using elevated_control::kWristPitchDeadband;
  using elevated_control::WristPitchDialNormalizedToVelocity;

  EXPECT_FLOAT_EQ(
      WristPitchDialNormalizedToVelocity(0.5f * kWristPitchDeadband), 0.0f);
  EXPECT_FLOAT_EQ(
      WristPitchDialNormalizedToVelocity(-0.5f * kWristPitchDeadband), 0.0f);
}
