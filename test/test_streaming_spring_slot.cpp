// Copyright (c) 2025 Elevate Robotics Inc

#include "elevated_control/arm_types.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace elevated_control {

TEST(SpringStreamingSlot, OnlyNanAccepted) {
  EXPECT_TRUE(IsSpringStreamingCommandSlotUnused(
      std::numeric_limits<float>::quiet_NaN()));
}

TEST(SpringStreamingSlot, ZeroAndNonNanRejected) {
  EXPECT_FALSE(IsSpringStreamingCommandSlotUnused(0.0f));
  EXPECT_FALSE(IsSpringStreamingCommandSlotUnused(-0.0f));
  EXPECT_FALSE(IsSpringStreamingCommandSlotUnused(1.0e-12f));
  EXPECT_FALSE(IsSpringStreamingCommandSlotUnused(-0.1f));
  EXPECT_FALSE(IsSpringStreamingCommandSlotUnused(42.0f));
  EXPECT_FALSE(IsSpringStreamingCommandSlotUnused(
      std::numeric_limits<float>::infinity()));
  EXPECT_FALSE(IsSpringStreamingCommandSlotUnused(
      -std::numeric_limits<float>::infinity()));
}

}  // namespace elevated_control
