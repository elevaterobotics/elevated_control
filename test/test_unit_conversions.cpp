#include <gtest/gtest.h>
#include <cmath>

#include "elevated_control/unit_conversions.hpp"

class UnitConversionTest : public ::testing::Test {
 protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(UnitConversionTest, ZeroTicksReturnsZero) {
  const int32_t ticks = 0;
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;

  const float result = elevated_control::InputTicksToOutputShaftRad(
      ticks, mechanical_reduction, encoder_resolution, 0);

  EXPECT_FLOAT_EQ(result, 0.0f);
}

TEST_F(UnitConversionTest, PositiveTicksReturnsCorrectRadians) {
  const int32_t ticks = 2560;
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;
  const float expected_radians = 2.0f * static_cast<float>(M_PI) / 170.0f;

  const float result = elevated_control::InputTicksToOutputShaftRad(
      ticks, mechanical_reduction, encoder_resolution, 0);

  EXPECT_NEAR(result, expected_radians, 1e-5f);
}

TEST_F(UnitConversionTest, NegativeTicksReturnsCorrectRadians) {
  const int32_t ticks = -2560;
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;
  const float expected_radians = -2.0f * static_cast<float>(M_PI) / 170.0f;

  const float result = elevated_control::InputTicksToOutputShaftRad(
      ticks, mechanical_reduction, encoder_resolution, 0);

  EXPECT_NEAR(result, expected_radians, 1e-5f);
}

TEST_F(UnitConversionTest, ZeroRadiansReturnsZeroTicks) {
  const float output_shaft_rad = 0.0f;
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;

  const int32_t result = elevated_control::OutputShaftRadToInputTicks(
      output_shaft_rad, mechanical_reduction, encoder_resolution);

  EXPECT_EQ(result, 0);
}

TEST_F(UnitConversionTest, PositiveRadiansReturnsCorrectTicks) {
  const float output_shaft_rad = 2.0f * static_cast<float>(M_PI) / 170.0f;
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;
  const int32_t expected_ticks = 2560;

  const int32_t result = elevated_control::OutputShaftRadToInputTicks(
      output_shaft_rad, mechanical_reduction, encoder_resolution);

  EXPECT_EQ(result, expected_ticks);
}

TEST_F(UnitConversionTest, NegativeRadiansReturnsCorrectTicks) {
  const float output_shaft_rad = -2.0f * static_cast<float>(M_PI) / 170.0f;
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;
  const int32_t expected_ticks = -2560;

  const int32_t result = elevated_control::OutputShaftRadToInputTicks(
      output_shaft_rad, mechanical_reduction, encoder_resolution);

  EXPECT_EQ(result, expected_ticks);
}

TEST_F(UnitConversionTest, SpringPotZeroTicksReturnsNegativeOffset) {
  const int32_t spring_pot_ticks = 0;
  const float expected_payload_kg = -5.8f;

  const float result =
      elevated_control::SpringPotTicksToPayloadKg(spring_pot_ticks);

  EXPECT_FLOAT_EQ(result, expected_payload_kg);
}

TEST_F(UnitConversionTest, SpringPotLargeTicksReturnsCorrectPayload) {
  const int32_t spring_pot_ticks = 1000;
  const float expected_payload_kg = 0.03f * 1000 - 5.8f;

  const float result =
      elevated_control::SpringPotTicksToPayloadKg(spring_pot_ticks);

  EXPECT_NEAR(result, expected_payload_kg, 1e-5f);
}

TEST_F(UnitConversionTest, TorquePerMilleZeroReturnsZero) {
  const int16_t torque_per_mille = 0;
  std::atomic<float> mechanical_reduction(170.0f);
  std::atomic<float> rated_torque(2.5f);

  const float result = elevated_control::TorquePerMilleToTorqueNm(
      torque_per_mille, mechanical_reduction, rated_torque);

  EXPECT_FLOAT_EQ(result, 0.0f);
}

TEST_F(UnitConversionTest, TorquePerMillePositiveReturnsCorrectNm) {
  const int16_t torque_per_mille = 1000;
  std::atomic<float> mechanical_reduction(170.0f);
  std::atomic<float> rated_torque(2.5f);
  const float expected_torque_nm = 1000.0f * 170.0f * 2.5f / 1000.0f;

  const float result = elevated_control::TorquePerMilleToTorqueNm(
      torque_per_mille, mechanical_reduction, rated_torque);

  EXPECT_NEAR(result, expected_torque_nm, 1e-2f);
}

TEST_F(UnitConversionTest, TorquePerMilleNegativeReturnsCorrectNm) {
  const int16_t torque_per_mille = -1000;
  std::atomic<float> mechanical_reduction(170.0f);
  std::atomic<float> rated_torque(2.5f);
  const float expected_torque_nm = -1000.0f * 170.0f * 2.5f / 1000.0f;

  const float result = elevated_control::TorquePerMilleToTorqueNm(
      torque_per_mille, mechanical_reduction, rated_torque);

  EXPECT_NEAR(result, expected_torque_nm, 1e-2f);
}

TEST_F(UnitConversionTest, RoundTripWithStartupWrapping) {
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;

  // Choose ticks that produce an unwrapped angle > PI to trigger -2*PI wrapping
  const int32_t original_ticks = 220000;

  const size_t joint_idx = 1;
  const float wrapped_rad = elevated_control::InputTicksToOutputShaftRad(
      original_ticks, mechanical_reduction, encoder_resolution, joint_idx);

  const float unwrapped_rad =
      (static_cast<float>(original_ticks) / encoder_resolution) * 2.0f *
      static_cast<float>(M_PI) / mechanical_reduction;
  EXPECT_NEAR(wrapped_rad, unwrapped_rad - 2.0f * static_cast<float>(M_PI),
              1e-3f);

  const int32_t round_tripped_ticks =
      elevated_control::OutputShaftRadToInputTicks(
          wrapped_rad, mechanical_reduction, encoder_resolution, joint_idx);

  EXPECT_NEAR(round_tripped_ticks, original_ticks, 1);
}

TEST_F(UnitConversionTest, RoundTripWithoutWrapping) {
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;

  const int32_t original_ticks = 2560;

  const size_t joint_idx = 2;
  const float rad = elevated_control::InputTicksToOutputShaftRad(
      original_ticks, mechanical_reduction, encoder_resolution, joint_idx);

  const int32_t round_tripped_ticks =
      elevated_control::OutputShaftRadToInputTicks(
          rad, mechanical_reduction, encoder_resolution, joint_idx);

  EXPECT_NEAR(round_tripped_ticks, original_ticks, 1);
}
