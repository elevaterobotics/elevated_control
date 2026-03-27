#include <gtest/gtest.h>

#include "elevated_control/joint_admittance.hpp"

namespace elevated_control {
namespace {

class JointAdmittanceTest : public ::testing::Test {
 protected:
  JointAdmittance admittance_;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(JointAdmittanceTest, InitWithValidParameters) {
  EXPECT_TRUE(admittance_.Init(1.0f, 2.0f));
  EXPECT_FLOAT_EQ(admittance_.inertia(), 1.0f);
  EXPECT_FLOAT_EQ(admittance_.damping(), 2.0f);
}

TEST_F(JointAdmittanceTest, InitWithLargeValidParameters) {
  EXPECT_TRUE(admittance_.Init(100.0f, 50.0f));
  EXPECT_FLOAT_EQ(admittance_.inertia(), 100.0f);
  EXPECT_FLOAT_EQ(admittance_.damping(), 50.0f);
}

TEST_F(JointAdmittanceTest, InitWithSmallValidParameters) {
  EXPECT_TRUE(admittance_.Init(0.01f, 0.01f));
  EXPECT_FLOAT_EQ(admittance_.inertia(), 0.01f);
  EXPECT_FLOAT_EQ(admittance_.damping(), 0.01f);
}

TEST_F(JointAdmittanceTest, InitWithZeroInertiaFails) {
  EXPECT_FALSE(admittance_.Init(0.0f, 1.0f));
}

TEST_F(JointAdmittanceTest, InitWithZeroDampingFails) {
  EXPECT_FALSE(admittance_.Init(1.0f, 0.0f));
}

TEST_F(JointAdmittanceTest, InitWithBothZeroFails) {
  EXPECT_FALSE(admittance_.Init(0.0f, 0.0f));
}

TEST_F(JointAdmittanceTest, InitWithNegativeInertiaFails) {
  EXPECT_FALSE(admittance_.Init(-1.0f, 1.0f));
}

TEST_F(JointAdmittanceTest, InitWithNegativeDampingFails) {
  EXPECT_FALSE(admittance_.Init(1.0f, -1.0f));
}

TEST_F(JointAdmittanceTest, InitWithBothNegativeFails) {
  EXPECT_FALSE(admittance_.Init(-1.0f, -2.0f));
}

TEST_F(JointAdmittanceTest, InitWithVerySmallInertiaFails) {
  EXPECT_FALSE(admittance_.Init(1e-4f, 1.0f));
}

TEST_F(JointAdmittanceTest, InitWithVerySmallDampingFails) {
  EXPECT_FALSE(admittance_.Init(1.0f, 1e-4f));
}

TEST_F(JointAdmittanceTest, InitCanBeCalledMultipleTimes) {
  EXPECT_TRUE(admittance_.Init(1.0f, 2.0f));
  EXPECT_FLOAT_EQ(admittance_.inertia(), 1.0f);
  EXPECT_FLOAT_EQ(admittance_.damping(), 2.0f);

  EXPECT_TRUE(admittance_.Init(3.0f, 4.0f));
  EXPECT_FLOAT_EQ(admittance_.inertia(), 3.0f);
  EXPECT_FLOAT_EQ(admittance_.damping(), 4.0f);
}

// ============================================================================
// Getter Tests
// ============================================================================

TEST_F(JointAdmittanceTest, InertiaGetterReturnsCorrectValue) {
  admittance_.Init(5.5f, 3.3f);
  EXPECT_FLOAT_EQ(admittance_.inertia(), 5.5f);
}

TEST_F(JointAdmittanceTest, DampingGetterReturnsCorrectValue) {
  admittance_.Init(5.5f, 3.3f);
  EXPECT_FLOAT_EQ(admittance_.damping(), 3.3f);
}

// ============================================================================
// Velocity Calculation Tests
// ============================================================================

TEST_F(JointAdmittanceTest, CalculateVelocityWithZeroInputs) {
  admittance_.Init(1.0f, 2.0f);
  float velocity = admittance_.CalculateVelocity(0.0f, 0.0f);
  EXPECT_FLOAT_EQ(velocity, 0.0f);
}

TEST_F(JointAdmittanceTest, CalculateVelocityWithOnlyExternalTorque) {
  admittance_.Init(1.0f, 2.0f);

  // raw = (10.0 - 1.0*0.0) / 2.0 = 5.0
  float velocity = admittance_.CalculateVelocity(0.0f, 10.0f);
  EXPECT_FLOAT_EQ(velocity, 5.0f);
}

TEST_F(JointAdmittanceTest, CalculateVelocityWithOnlyAcceleration) {
  admittance_.Init(2.0f, 4.0f);

  // raw = (0.0 - 2.0*3.0) / 4.0 = -1.5
  float velocity = admittance_.CalculateVelocity(3.0f, 0.0f);
  EXPECT_FLOAT_EQ(velocity, -1.5f);
}

TEST_F(JointAdmittanceTest, CalculateVelocityWithBothInputs) {
  admittance_.Init(2.0f, 5.0f);

  // raw = (20.0 - 2.0*3.0) / 5.0 = 2.8
  float velocity = admittance_.CalculateVelocity(3.0f, 20.0f);
  EXPECT_FLOAT_EQ(velocity, 2.8f);
}

TEST_F(JointAdmittanceTest, CalculateVelocityWithNegativeAcceleration) {
  admittance_.Init(1.0f, 2.0f);

  // raw = (5.0 - 1.0*(-2.0)) / 2.0 = 3.5
  float velocity = admittance_.CalculateVelocity(-2.0f, 5.0f);
  EXPECT_FLOAT_EQ(velocity, 3.5f);
}

TEST_F(JointAdmittanceTest, CalculateVelocityWithNegativeExternalTorque) {
  admittance_.Init(1.0f, 2.0f);

  // raw = (-10.0 - 0.0) / 2.0 = -5.0
  float velocity = admittance_.CalculateVelocity(0.0f, -10.0f);
  EXPECT_FLOAT_EQ(velocity, -5.0f);
}

TEST_F(JointAdmittanceTest, CalculateVelocityWithAllNegativeInputs) {
  admittance_.Init(2.0f, 4.0f);

  // raw = (-8.0 - 2.0*(-3.0)) / 4.0 = -0.5
  float velocity = admittance_.CalculateVelocity(-3.0f, -8.0f);
  EXPECT_FLOAT_EQ(velocity, -0.5f);
}

TEST_F(JointAdmittanceTest, CalculateVelocityWithDifferentParameters) {
  admittance_.Init(10.0f, 20.0f);

  // raw = (100.0 - 10.0*5.0) / 20.0 = 2.5
  float velocity = admittance_.CalculateVelocity(5.0f, 100.0f);
  EXPECT_FLOAT_EQ(velocity, 2.5f);
}

TEST_F(JointAdmittanceTest, RepeatedCallsAreDeterministic) {
  admittance_.Init(1.0f, 2.0f);

  float velocity1 = admittance_.CalculateVelocity(1.0f, 5.0f);
  float velocity2 = admittance_.CalculateVelocity(1.0f, 5.0f);
  float velocity3 = admittance_.CalculateVelocity(1.0f, 5.0f);

  EXPECT_FLOAT_EQ(velocity1, 2.0f);
  EXPECT_FLOAT_EQ(velocity2, 2.0f);
  EXPECT_FLOAT_EQ(velocity3, 2.0f);
}

TEST_F(JointAdmittanceTest, CalculateVelocityWithSmallDamping) {
  admittance_.Init(1.0f, 0.01f);

  // raw = (1.0 - 0.0) / 0.01 = 100.0
  float velocity = admittance_.CalculateVelocity(0.0f, 1.0f);
  EXPECT_FLOAT_EQ(velocity, 100.0f);
}

TEST_F(JointAdmittanceTest, CalculateVelocityWithLargeDamping) {
  admittance_.Init(1.0f, 100.0f);

  // raw = (10.0 - 0.0) / 100.0 = 0.1
  float velocity = admittance_.CalculateVelocity(0.0f, 10.0f);
  EXPECT_FLOAT_EQ(velocity, 0.1f);
}

TEST_F(JointAdmittanceTest, CalculateVelocityPhysicalExample) {
  admittance_.Init(0.5f, 10.0f);

  // raw = (5.0 - 0.5*2.0) / 10.0 = 0.4
  float velocity = admittance_.CalculateVelocity(2.0f, 5.0f);
  EXPECT_FLOAT_EQ(velocity, 0.4f);
}

}  // namespace
}  // namespace elevated_control
