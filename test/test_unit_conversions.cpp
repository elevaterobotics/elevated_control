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
  std::atomic<float> wrap_value{0.0f};
  elevated_control::ComputeStartupWrapOffset(ticks, mechanical_reduction,
                                             encoder_resolution, wrap_value);

  const float result = elevated_control::InputTicksToOutputShaftRad(
      ticks, mechanical_reduction, encoder_resolution, wrap_value);

  EXPECT_FLOAT_EQ(result, 0.0f);
}

TEST_F(UnitConversionTest, PositiveTicksReturnsCorrectRadians) {
  const int32_t ticks = 2560;
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;
  const float expected_radians = 2.0f * static_cast<float>(M_PI) / 170.0f;
  std::atomic<float> wrap_value{0.0f};
  elevated_control::ComputeStartupWrapOffset(ticks, mechanical_reduction,
                                             encoder_resolution, wrap_value);

  const float result = elevated_control::InputTicksToOutputShaftRad(
      ticks, mechanical_reduction, encoder_resolution, wrap_value);

  EXPECT_NEAR(result, expected_radians, 1e-5f);
}

TEST_F(UnitConversionTest, NegativeTicksReturnsCorrectRadians) {
  const int32_t ticks = -2560;
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;
  const float expected_radians = -2.0f * static_cast<float>(M_PI) / 170.0f;
  std::atomic<float> wrap_value{0.0f};
  elevated_control::ComputeStartupWrapOffset(ticks, mechanical_reduction,
                                             encoder_resolution, wrap_value);

  const float result = elevated_control::InputTicksToOutputShaftRad(
      ticks, mechanical_reduction, encoder_resolution, wrap_value);

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
  const int32_t original_ticks = 220000;

  std::atomic<float> wrap_value{0.0f};
  elevated_control::ComputeStartupWrapOffset(original_ticks, mechanical_reduction,
                                             encoder_resolution, wrap_value);

  const float wrapped_rad = elevated_control::InputTicksToOutputShaftRad(
      original_ticks, mechanical_reduction, encoder_resolution, wrap_value);

  const float unwrapped_rad =
      (static_cast<float>(original_ticks) / encoder_resolution) * 2.0f *
      static_cast<float>(M_PI) / mechanical_reduction;
  EXPECT_NEAR(wrapped_rad, unwrapped_rad - 2.0f * static_cast<float>(M_PI),
              1e-3f);

  const int32_t round_tripped_ticks =
      elevated_control::OutputShaftRadToInputTicks(
          wrapped_rad, mechanical_reduction, encoder_resolution, wrap_value);

  EXPECT_NEAR(round_tripped_ticks, original_ticks, 1);
}

TEST_F(UnitConversionTest, RoundTripWithoutWrapping) {
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;
  const int32_t original_ticks = 2560;

  std::atomic<float> wrap_value{0.0f};
  elevated_control::ComputeStartupWrapOffset(original_ticks, mechanical_reduction,
                                             encoder_resolution, wrap_value);

  const float rad = elevated_control::InputTicksToOutputShaftRad(
      original_ticks, mechanical_reduction, encoder_resolution, wrap_value);

  const int32_t round_tripped_ticks =
      elevated_control::OutputShaftRadToInputTicks(
          rad, mechanical_reduction, encoder_resolution, wrap_value);

  EXPECT_NEAR(round_tripped_ticks, original_ticks, 1);
}

// --- VelocityValueToOutputShaftRadPerS tests ---

TEST_F(UnitConversionTest, VelocityValueToRadPerS_ZeroMilliRpm) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0xFDB44700u);
  const float result = elevated_control::VelocityValueToOutputShaftRadPerS(
      0, si_velocity_unit, 170.0f, 2560);
  EXPECT_NEAR(result, 0.0f, 1e-5f);
}

TEST_F(UnitConversionTest, VelocityValueToRadPerS_PositiveMilliRpm) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0xFDB44700u);
  const float result = elevated_control::VelocityValueToOutputShaftRadPerS(
      60000, si_velocity_unit, 170.0f, 2560);
  EXPECT_NEAR(result, 2.0f * static_cast<float>(M_PI) / 170.0f, 1e-5f);
}

TEST_F(UnitConversionTest, VelocityValueToRadPerS_NegativeMilliRpm) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0xFDB44700u);
  const float result = elevated_control::VelocityValueToOutputShaftRadPerS(
      -60000, si_velocity_unit, 170.0f, 2560);
  EXPECT_NEAR(result, -2.0f * static_cast<float>(M_PI) / 170.0f, 1e-5f);
}

TEST_F(UnitConversionTest, VelocityValueToRadPerS_ArbitraryMilliRpm) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0xFDB44700u);
  const float expected =
      1000.0f * 2.0f * static_cast<float>(M_PI) / (60.0f * 1000.0f * 170.0f);
  const float result = elevated_control::VelocityValueToOutputShaftRadPerS(
      1000, si_velocity_unit, 170.0f, 2560);
  EXPECT_NEAR(result, expected, 1e-6f);
}

TEST_F(UnitConversionTest, VelocityValueToRadPerS_FallbackToTicks) {
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;
  const int32_t velocity_value = 5000;
  const int32_t si_velocity_unit = 0;

  const float result = elevated_control::VelocityValueToOutputShaftRadPerS(
      velocity_value, si_velocity_unit, mechanical_reduction,
      encoder_resolution);
  const float expected =
      elevated_control::InputTicksVelocityToOutputShaftRadPerS(
          velocity_value, mechanical_reduction, encoder_resolution);

  EXPECT_NEAR(result, expected, 1e-4f);
}

// --- OutputShaftRadPerSToVelocityValue tests ---

TEST_F(UnitConversionTest, RadPerSToVelocityValue_ZeroMilliRpm) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0xFDB44700u);
  const int32_t result =
      elevated_control::OutputShaftRadPerSToVelocityValue(0.0f, si_velocity_unit,
                                                          170.0f, 2560);
  EXPECT_EQ(result, 0);
}

TEST_F(UnitConversionTest, RadPerSToVelocityValue_PositiveMilliRpm) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0xFDB44700u);
  const int32_t result = elevated_control::OutputShaftRadPerSToVelocityValue(
      2.0f * static_cast<float>(M_PI) / 170.0f, si_velocity_unit, 170.0f, 2560);
  EXPECT_NEAR(result, 60000, 1);
}

TEST_F(UnitConversionTest, RadPerSToVelocityValue_NegativeMilliRpm) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0xFDB44700u);
  const int32_t result = elevated_control::OutputShaftRadPerSToVelocityValue(
      -2.0f * static_cast<float>(M_PI) / 170.0f, si_velocity_unit, 170.0f, 2560);
  EXPECT_NEAR(result, -60000, 1);
}

TEST_F(UnitConversionTest, RadPerSToVelocityValue_ArbitraryMilliRpm) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0xFDB44700u);
  const int32_t expected = static_cast<int32_t>(
      1.0f * 170.0f * 60.0f * 1000.0f / (2.0f * static_cast<float>(M_PI)));
  const int32_t result = elevated_control::OutputShaftRadPerSToVelocityValue(
      1.0f, si_velocity_unit, 170.0f, 2560);
  EXPECT_NEAR(result, expected, 1);
}

TEST_F(UnitConversionTest, VelocityMilliRpmRoundTrip) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0xFDB44700u);
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;
  const float original_rad_per_s = 1.23f;

  const int32_t velocity_value =
      elevated_control::OutputShaftRadPerSToVelocityValue(
          original_rad_per_s, si_velocity_unit, mechanical_reduction,
          encoder_resolution);
  const float round_tripped =
      elevated_control::VelocityValueToOutputShaftRadPerS(
          velocity_value, si_velocity_unit, mechanical_reduction,
          encoder_resolution);

  EXPECT_NEAR(round_tripped, original_rad_per_s, 2e-4f);
}

// --- VelocityValueToOutputShaftRadPerS RPM tests ---

TEST_F(UnitConversionTest, VelocityValueToRadPerS_ZeroRpm) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0x00B44700u);
  const float result = elevated_control::VelocityValueToOutputShaftRadPerS(
      0, si_velocity_unit, 170.0f, 2560);
  EXPECT_NEAR(result, 0.0f, 1e-5f);
}

TEST_F(UnitConversionTest, VelocityValueToRadPerS_PositiveRpm) {
  // 60 RPM motor-shaft = 1 rev/s = 2*PI rad/s motor-shaft
  // output-shaft = 2*PI / 170 rad/s
  const int32_t si_velocity_unit = static_cast<int32_t>(0x00B44700u);
  const float result = elevated_control::VelocityValueToOutputShaftRadPerS(
      60, si_velocity_unit, 170.0f, 2560);
  EXPECT_NEAR(result, 2.0f * static_cast<float>(M_PI) / 170.0f, 1e-5f);
}

TEST_F(UnitConversionTest, VelocityValueToRadPerS_NegativeRpm) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0x00B44700u);
  const float result = elevated_control::VelocityValueToOutputShaftRadPerS(
      -60, si_velocity_unit, 170.0f, 2560);
  EXPECT_NEAR(result, -2.0f * static_cast<float>(M_PI) / 170.0f, 1e-5f);
}

// --- OutputShaftRadPerSToVelocityValue RPM tests ---

TEST_F(UnitConversionTest, RadPerSToVelocityValue_ZeroRpm) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0x00B44700u);
  const int32_t result =
      elevated_control::OutputShaftRadPerSToVelocityValue(0.0f, si_velocity_unit,
                                                          170.0f, 2560);
  EXPECT_EQ(result, 0);
}

TEST_F(UnitConversionTest, RadPerSToVelocityValue_PositiveRpm) {
  // output-shaft 2*PI/170 rad/s -> motor-shaft 60 RPM
  const int32_t si_velocity_unit = static_cast<int32_t>(0x00B44700u);
  const int32_t result = elevated_control::OutputShaftRadPerSToVelocityValue(
      2.0f * static_cast<float>(M_PI) / 170.0f, si_velocity_unit, 170.0f, 2560);
  EXPECT_NEAR(result, 60, 1);
}

TEST_F(UnitConversionTest, RadPerSToVelocityValue_NegativeRpm) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0x00B44700u);
  const int32_t result = elevated_control::OutputShaftRadPerSToVelocityValue(
      -2.0f * static_cast<float>(M_PI) / 170.0f, si_velocity_unit, 170.0f, 2560);
  EXPECT_NEAR(result, -60, 1);
}

TEST_F(UnitConversionTest, VelocityRpmRoundTrip) {
  const int32_t si_velocity_unit = static_cast<int32_t>(0x00B44700u);
  const float mechanical_reduction = 170.0f;
  const uint32_t encoder_resolution = 2560;
  const float original_rad_per_s = 1.23f;

  const int32_t velocity_value =
      elevated_control::OutputShaftRadPerSToVelocityValue(
          original_rad_per_s, si_velocity_unit, mechanical_reduction,
          encoder_resolution);
  const float round_tripped =
      elevated_control::VelocityValueToOutputShaftRadPerS(
          velocity_value, si_velocity_unit, mechanical_reduction,
          encoder_resolution);

  // int32 truncation: 1 RPM ≈ 0.1047 rad/s at motor, /170 ≈ 6.2e-4 rad/s at output
  EXPECT_NEAR(round_tripped, original_rad_per_s, 7e-4f);
}
