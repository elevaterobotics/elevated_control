#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "elevated_control/constants_arm.hpp"
#include "elevated_control/dial_normalization.hpp"

namespace {

constexpr float kEps = 1e-6f;

float ExpectedWristPitchPositiveRamp(float normalized_dial) {
  const float v_min = 0.0f;
  const float v_max = elevated_control::kMaxWristPitchDialVelocity;
  const float slope =
      (v_max - v_min) / (1.0f - elevated_control::kWristPitchDeadband);
  return slope * (normalized_dial - elevated_control::kWristPitchDeadband) +
         v_min;
}

float ExpectedWristPitchNegativeRamp(float normalized_dial) {
  const float v_min = 0.0f;
  const float v_max = elevated_control::kMaxWristPitchDialVelocity;
  const float slope =
      (v_max - v_min) / (1.0f - elevated_control::kWristPitchDeadband);
  return slope * (normalized_dial + elevated_control::kWristPitchDeadband) -
         v_min;
}

float ExpectedWristRollPositiveRamp(float normalized_dial) {
  const float slope = elevated_control::kMaxWristRollDialVelocity /
                      (1.0f - elevated_control::kWristRollDeadband);
  return slope * (normalized_dial - elevated_control::kWristRollDeadband);
}

float ExpectedWristRollNegativeRamp(float normalized_dial) {
  const float slope = elevated_control::kMaxWristRollDialVelocity /
                      (1.0f - elevated_control::kWristRollDeadband);
  return slope * (normalized_dial + 1.0f) -
         elevated_control::kMaxWristRollDialVelocity;
}

}  // namespace

TEST(DialNormalization, WristPitchZeroAtNeutral) {
  elevated_control::DialActivationState state;

  EXPECT_FALSE(
      elevated_control::UpdateWristPitchDialActivationState(0.0f, state));
  EXPECT_FALSE(state.is_active);
}

TEST(DialNormalization, WristPitchEngagesAtDeadbandBoundary) {
  using elevated_control::kMaxWristPitchDialVelocity;
  using elevated_control::kWristPitchDeadband;
  using elevated_control::NormalizeWristPitchDialToVelocity;
  using elevated_control::UpdateWristPitchDialActivationState;
  elevated_control::DialActivationState positive_state;
  elevated_control::DialActivationState negative_state;

  EXPECT_TRUE(
      UpdateWristPitchDialActivationState(kWristPitchDeadband, positive_state));
  EXPECT_NEAR(NormalizeWristPitchDialToVelocity(kWristPitchDeadband), 0.0f,
              kEps);
  EXPECT_TRUE(UpdateWristPitchDialActivationState(-kWristPitchDeadband,
                                                  negative_state));
  EXPECT_NEAR(NormalizeWristPitchDialToVelocity(-kWristPitchDeadband), 0.0f,
              kEps);
  EXPECT_NEAR(NormalizeWristPitchDialToVelocity(1.0f),
              kMaxWristPitchDialVelocity, kEps);
  EXPECT_NEAR(NormalizeWristPitchDialToVelocity(-1.0f),
              -kMaxWristPitchDialVelocity, kEps);
}

TEST(DialNormalization, WristPitchSaturatesOutsideUnitRange) {
  using elevated_control::kMaxWristPitchDialVelocity;
  using elevated_control::NormalizeWristPitchDialToVelocity;

  EXPECT_FLOAT_EQ(NormalizeWristPitchDialToVelocity(1.5f),
                  kMaxWristPitchDialVelocity);
  EXPECT_FLOAT_EQ(NormalizeWristPitchDialToVelocity(-1.5f),
                  -kMaxWristPitchDialVelocity);
}

TEST(DialNormalization, WristPitchInteriorPointsMatchLinearRamp) {
  using elevated_control::NormalizeWristPitchDialToVelocity;

  constexpr float kMidPos = 0.5f;
  constexpr float kMidNeg = -0.5f;
  EXPECT_NEAR(NormalizeWristPitchDialToVelocity(kMidPos),
              ExpectedWristPitchPositiveRamp(kMidPos), kEps);
  EXPECT_NEAR(NormalizeWristPitchDialToVelocity(kMidNeg),
              ExpectedWristPitchNegativeRamp(kMidNeg), kEps);
}

TEST(DialNormalization, WristPitchIsZeroInsideDeadband) {
  using elevated_control::kWristPitchDeadband;
  using elevated_control::NormalizeWristPitchDialToVelocity;

  EXPECT_FLOAT_EQ(NormalizeWristPitchDialToVelocity(0.0f), 0.0f);
  EXPECT_FLOAT_EQ(
      NormalizeWristPitchDialToVelocity(0.5f * kWristPitchDeadband), 0.0f);
  EXPECT_FLOAT_EQ(
      NormalizeWristPitchDialToVelocity(-0.5f * kWristPitchDeadband), 0.0f);
}

TEST(DialNormalization, WristPitchHoldsOutputInsideHysteresisBandUntilRelease) {
  using elevated_control::kWristPitchDeadband;
  using elevated_control::kWristPitchDeadbandHysteresis;
  using elevated_control::NormalizeWristPitchDialToVelocity;
  using elevated_control::UpdateWristPitchDialActivationState;
  elevated_control::DialActivationState positive_state;
  elevated_control::DialActivationState negative_state;

  const float positive_hold =
      kWristPitchDeadband - 0.5f * kWristPitchDeadbandHysteresis;
  const float negative_hold = -positive_hold;
  const float release_threshold =
      kWristPitchDeadband - kWristPitchDeadbandHysteresis;

  EXPECT_TRUE(
      UpdateWristPitchDialActivationState(kWristPitchDeadband, positive_state));
  EXPECT_NEAR(NormalizeWristPitchDialToVelocity(kWristPitchDeadband), 0.0f,
              kEps);
  EXPECT_TRUE(
      UpdateWristPitchDialActivationState(positive_hold, positive_state));
  EXPECT_FLOAT_EQ(NormalizeWristPitchDialToVelocity(positive_hold), 0.0f);
  EXPECT_TRUE(positive_state.is_active);
  EXPECT_FALSE(
      UpdateWristPitchDialActivationState(release_threshold, positive_state));
  EXPECT_FALSE(positive_state.is_active);

  EXPECT_TRUE(UpdateWristPitchDialActivationState(-kWristPitchDeadband,
                                                  negative_state));
  EXPECT_NEAR(NormalizeWristPitchDialToVelocity(-kWristPitchDeadband), 0.0f,
              kEps);
  EXPECT_TRUE(
      UpdateWristPitchDialActivationState(negative_hold, negative_state));
  EXPECT_FLOAT_EQ(NormalizeWristPitchDialToVelocity(negative_hold), 0.0f);
  EXPECT_TRUE(negative_state.is_active);
  EXPECT_FALSE(UpdateWristPitchDialActivationState(-release_threshold,
                                                   negative_state));
  EXPECT_FALSE(negative_state.is_active);
}

TEST(DialNormalization, WristPitchResetClearsLatchedActivity) {
  using elevated_control::kWristPitchDeadband;
  using elevated_control::kWristPitchDeadbandHysteresis;
  using elevated_control::UpdateWristPitchDialActivationState;
  elevated_control::DialActivationState state;

  EXPECT_TRUE(
      UpdateWristPitchDialActivationState(kWristPitchDeadband, state));
  EXPECT_TRUE(state.is_active);

  state.Reset();
  EXPECT_FALSE(state.is_active);
  EXPECT_FALSE(UpdateWristPitchDialActivationState(
      kWristPitchDeadband - 0.5f * kWristPitchDeadbandHysteresis, state));
  EXPECT_FALSE(state.is_active);
}

TEST(DialNormalization, WristPitchActivationStateSequence) {
  using elevated_control::kWristPitchDeadband;
  using elevated_control::kWristPitchDeadbandHysteresis;
  using elevated_control::NormalizeWristPitchDialToVelocity;
  using elevated_control::UpdateWristPitchDialActivationState;
  elevated_control::DialActivationState state;

  const float hold_sample =
      kWristPitchDeadband - 0.5f * kWristPitchDeadbandHysteresis;
  const float release_sample =
      kWristPitchDeadband - kWristPitchDeadbandHysteresis;
  const std::array<float, 6> samples = {0.0f,
                                        kWristPitchDeadband,
                                        hold_sample,
                                        release_sample,
                                        -kWristPitchDeadband,
                                        -release_sample};
  const std::array<bool, 6> expected_active = {false, true, true,
                                               false, true, false};

  for (std::size_t i = 0; i < samples.size(); ++i) {
    const bool active = UpdateWristPitchDialActivationState(samples[i], state);
    const float velocity =
        active ? NormalizeWristPitchDialToVelocity(samples[i]) : 0.0f;

    EXPECT_EQ(active, expected_active[i]);
    EXPECT_EQ(state.is_active, expected_active[i]);
    if (std::abs(samples[i]) < kWristPitchDeadband) {
      EXPECT_FLOAT_EQ(velocity, 0.0f);
    }
  }
}

TEST(DialNormalization, WristRollIsZeroInsideDeadband) {
  using elevated_control::kWristRollDeadband;
  using elevated_control::NormalizeWristRollDialToVelocity;

  EXPECT_FLOAT_EQ(NormalizeWristRollDialToVelocity(0.0f), 0.0f);
  EXPECT_FLOAT_EQ(
      NormalizeWristRollDialToVelocity(0.5f * kWristRollDeadband), 0.0f);
  EXPECT_FLOAT_EQ(
      NormalizeWristRollDialToVelocity(-0.5f * kWristRollDeadband), 0.0f);
}

TEST(DialNormalization, WristRollInteriorPointsMatchLinearRamp) {
  using elevated_control::NormalizeWristRollDialToVelocity;

  constexpr float kMidPos = 0.5f;
  constexpr float kMidNeg = -0.5f;
  EXPECT_NEAR(NormalizeWristRollDialToVelocity(kMidPos),
              ExpectedWristRollPositiveRamp(kMidPos), kEps);
  EXPECT_NEAR(NormalizeWristRollDialToVelocity(kMidNeg),
              ExpectedWristRollNegativeRamp(kMidNeg), kEps);
}

TEST(DialNormalization, WristRollSaturatesOutsideUnitRange) {
  using elevated_control::kMaxWristRollDialVelocity;
  using elevated_control::NormalizeWristRollDialToVelocity;

  EXPECT_FLOAT_EQ(NormalizeWristRollDialToVelocity(1.5f),
                  kMaxWristRollDialVelocity);
  EXPECT_FLOAT_EQ(NormalizeWristRollDialToVelocity(-1.5f),
                  -kMaxWristRollDialVelocity);
}
