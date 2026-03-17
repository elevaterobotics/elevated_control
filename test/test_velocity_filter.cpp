#include <gtest/gtest.h>

#include "elevated_control/velocity_filter.hpp"

using elevated_control::VelocityFilter;

TEST(VelocityFilterTest, InitialOutputFromZeroState) {
  const float alpha = 0.2f;
  VelocityFilter filter(alpha);
  float output = filter.Filter(10.0f);
  EXPECT_FLOAT_EQ(output, (1.0f - alpha) * 10.0f);
}

TEST(VelocityFilterTest, ExponentialSmoothingSequence) {
  const float alpha = 0.5f;
  VelocityFilter filter(alpha);

  float o1 = filter.Filter(10.0f);
  float o2 = filter.Filter(10.0f);
  float o3 = filter.Filter(10.0f);
  float o4 = filter.Filter(10.0f);

  EXPECT_FLOAT_EQ(o1, 5.0f);
  EXPECT_FLOAT_EQ(o2, 7.5f);
  EXPECT_FLOAT_EQ(o3, 8.75f);
  EXPECT_FLOAT_EQ(o4, 9.375f);
}

TEST(VelocityFilterTest, ResetClearsStateToZero) {
  const float alpha = 0.3f;
  VelocityFilter filter(alpha);

  (void)filter.Filter(10.0f);
  (void)filter.Filter(10.0f);

  filter.Reset();

  float after_reset = filter.Filter(10.0f);
  EXPECT_FLOAT_EQ(after_reset, (1.0f - alpha) * 10.0f);
}

TEST(VelocityFilterTest, AlphaZeroPassThrough) {
  VelocityFilter filter(0.0f);
  EXPECT_FLOAT_EQ(filter.Filter(3.0f), 3.0f);
  EXPECT_FLOAT_EQ(filter.Filter(-2.5f), -2.5f);
  EXPECT_FLOAT_EQ(filter.Filter(0.0f), 0.0f);
}

TEST(VelocityFilterTest, AlphaOneHoldsPreviousOutput) {
  VelocityFilter filter(1.0f);
  EXPECT_FLOAT_EQ(filter.Filter(3.0f), 0.0f);
  EXPECT_FLOAT_EQ(filter.Filter(100.0f), 0.0f);

  filter.Reset();
  EXPECT_FLOAT_EQ(filter.Filter(-50.0f), 0.0f);
}

TEST(VelocityFilterTest, HandlesNegativeAndChangingInputs) {
  const float alpha = 0.4f;
  VelocityFilter filter(alpha);

  float o1 = filter.Filter(5.0f);
  EXPECT_FLOAT_EQ(o1, 3.0f);

  float o2 = filter.Filter(-5.0f);
  EXPECT_FLOAT_EQ(o2, -1.8f);

  float o3 = filter.Filter(0.0f);
  EXPECT_FLOAT_EQ(o3, -0.72f);
}
