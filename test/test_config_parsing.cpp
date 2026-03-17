#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <limits>

#include "elevated_control/config_parsing.hpp"

class ConfigParsingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    joint_names_ = {"joint1", "joint2"};

    test_dir_ =
        std::filesystem::temp_directory_path() / "config_parsing_test";
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override {
    if (std::filesystem::exists(test_dir_)) {
      std::filesystem::remove_all(test_dir_);
    }
  }

  std::string CreateTestYamlFile(const std::string& filename,
                                 const std::string& content) {
    std::string filepath = (test_dir_ / filename).string();
    std::ofstream file(filepath);
    file << content;
    file.close();
    return filepath;
  }

  std::vector<std::string> joint_names_;
  std::filesystem::path test_dir_;
};

// -- ParseJointLimits tests --

TEST_F(ConfigParsingTest, ParseJointLimitsValidFile) {
  std::string yaml_content = R"(
joint_limits:
  joint1:
    has_position_limits: true
    min_position: -1.5
    max_position: 1.5
  joint2:
    has_position_limits: false
)";

  std::string filepath =
      CreateTestYamlFile("valid_joint_limits.yaml", yaml_content);

  auto config =
      elevated_control::ParseJointLimits(filepath, joint_names_);

  EXPECT_EQ(config.has_position_limits.size(), 2u);
  EXPECT_EQ(config.min_position_limits.size(), 2u);
  EXPECT_EQ(config.max_position_limits.size(), 2u);

  EXPECT_TRUE(config.has_position_limits[0]);
  EXPECT_FLOAT_EQ(config.min_position_limits[0], -1.5f);
  EXPECT_FLOAT_EQ(config.max_position_limits[0], 1.5f);

  EXPECT_FALSE(config.has_position_limits[1]);
  EXPECT_FLOAT_EQ(config.min_position_limits[1],
                   -std::numeric_limits<float>::max());
  EXPECT_FLOAT_EQ(config.max_position_limits[1],
                   std::numeric_limits<float>::max());
}

TEST_F(ConfigParsingTest, ParseJointLimitsInvalidLimits) {
  std::string yaml_content = R"(
joint_limits:
  joint1:
    has_position_limits: true
    min_position: 2.0
    max_position: 1.0
)";

  std::string filepath =
      CreateTestYamlFile("invalid_joint_limits.yaml", yaml_content);

  auto config =
      elevated_control::ParseJointLimits(filepath, joint_names_);

  EXPECT_TRUE(config.has_position_limits.empty());
}

TEST_F(ConfigParsingTest, ParseJointLimitsNonExistentFile) {
  std::string filepath = (test_dir_ / "non_existent.yaml").string();

  auto config =
      elevated_control::ParseJointLimits(filepath, joint_names_);

  EXPECT_TRUE(config.has_position_limits.empty());
}

TEST_F(ConfigParsingTest, ParseJointLimitsMissingJointLimitsSection) {
  std::string yaml_content = R"(
other_section:
  some_value: 123
)";

  std::string filepath =
      CreateTestYamlFile("missing_joint_limits.yaml", yaml_content);

  auto config =
      elevated_control::ParseJointLimits(filepath, joint_names_);

  EXPECT_TRUE(config.has_position_limits.empty());
}

TEST_F(ConfigParsingTest, ParseJointLimitsInvalidYaml) {
  std::string yaml_content = R"(
joint_limits:
  joint1:
    has_position_limits: true
    min_position: [invalid, yaml]
)";

  std::string filepath =
      CreateTestYamlFile("invalid_yaml.yaml", yaml_content);

  auto config =
      elevated_control::ParseJointLimits(filepath, joint_names_);

  EXPECT_TRUE(config.has_position_limits.empty());
}

// -- ParseElevateConfig tests --

TEST_F(ConfigParsingTest, ParseElevateConfigValidFile) {
  std::string yaml_content = R"(
button_config:
  analog_input_midpoint: 2048
  wrist_dial_min: 100
  wrist_dial_max: 3000
spring_setpoints:
  unloaded: 32000
  loaded: 50000
torque_sign:
  yaw1: 1
  yaw2: -1
  wrist_yaw: 1
)";

  std::string filepath =
      CreateTestYamlFile("valid_elevate_config.yaml", yaml_content);

  auto config = elevated_control::ParseElevateConfig(filepath);

  EXPECT_EQ(config.button_config.analog_input_midpoint, 2048);
  EXPECT_EQ(config.button_config.wrist_dial_min, 100);
  EXPECT_EQ(config.button_config.wrist_dial_max, 3000);

  EXPECT_EQ(config.spring_setpoints.spring_setpoint_unloaded, 32000);
  EXPECT_EQ(config.spring_setpoints.spring_setpoint_loaded, 50000);
}

TEST_F(ConfigParsingTest, ParseElevateConfigMissingButtonConfig) {
  std::string yaml_content = R"(
spring_setpoints:
  unloaded: 32000
  loaded: 50000
torque_sign:
  yaw1: 1
  yaw2: -1
  wrist_yaw: 1
)";

  std::string filepath =
      CreateTestYamlFile("missing_button_config.yaml", yaml_content);

  auto config = elevated_control::ParseElevateConfig(filepath);

  EXPECT_EQ(config.button_config.analog_input_midpoint, 0);
  EXPECT_EQ(config.button_config.wrist_dial_min, 0);
  EXPECT_EQ(config.button_config.wrist_dial_max, 0);
}

TEST_F(ConfigParsingTest, ParseElevateConfigMissingSpringSetpoints) {
  std::string yaml_content = R"(
button_config:
  analog_input_midpoint: 2048
  wrist_dial_min: 100
  wrist_dial_max: 3000
torque_sign:
  yaw1: 1
  yaw2: -1
  wrist_yaw: 1
)";

  std::string filepath =
      CreateTestYamlFile("missing_spring_setpoints.yaml", yaml_content);

  auto config = elevated_control::ParseElevateConfig(filepath);

  EXPECT_EQ(config.spring_setpoints.spring_setpoint_unloaded, 0);
  EXPECT_EQ(config.spring_setpoints.spring_setpoint_loaded, 0);
}

TEST_F(ConfigParsingTest, ParseElevateConfigNonExistentFile) {
  std::string filepath = (test_dir_ / "non_existent_elevate.yaml").string();

  auto config = elevated_control::ParseElevateConfig(filepath);

  EXPECT_EQ(config.button_config.analog_input_midpoint, 0);
  EXPECT_EQ(config.spring_setpoints.spring_setpoint_unloaded, 0);
}

// -- ValidateJointLimits tests --

TEST_F(ConfigParsingTest, ValidateJointLimitsValidConfig) {
  elevated_control::JointLimitsConfig config;
  config.has_position_limits = {true, false};
  config.min_position_limits = {-1.0f, -std::numeric_limits<float>::max()};
  config.max_position_limits = {1.0f, std::numeric_limits<float>::max()};

  bool result = elevated_control::ValidateJointLimits(config);

  EXPECT_TRUE(result);
}

TEST_F(ConfigParsingTest, ValidateJointLimitsEmptyConfig) {
  elevated_control::JointLimitsConfig config;

  bool result = elevated_control::ValidateJointLimits(config);

  EXPECT_FALSE(result);
}

// -- ValidateButtonConfig tests --

TEST_F(ConfigParsingTest, ValidateButtonConfigValidConfig) {
  elevated_control::ButtonConfig config;
  config.analog_input_midpoint = 2048;
  config.wrist_dial_min = 100;
  config.wrist_dial_max = 3000;

  bool result = elevated_control::ValidateButtonConfig(config);

  EXPECT_TRUE(result);
}

TEST_F(ConfigParsingTest, ValidateButtonConfigInvalidDialRange) {
  elevated_control::ButtonConfig config;
  config.analog_input_midpoint = 2048;
  config.wrist_dial_min = 3000;
  config.wrist_dial_max = 100;

  bool result = elevated_control::ValidateButtonConfig(config);

  EXPECT_FALSE(result);
}

TEST_F(ConfigParsingTest, ValidateButtonConfigNegativeValues) {
  elevated_control::ButtonConfig config;
  config.analog_input_midpoint = -100;
  config.wrist_dial_min = 100;
  config.wrist_dial_max = 3000;

  bool result = elevated_control::ValidateButtonConfig(config);

  EXPECT_FALSE(result);
}

TEST_F(ConfigParsingTest, ValidateButtonConfigZeroValues) {
  elevated_control::ButtonConfig config;
  config.analog_input_midpoint = 0;
  config.wrist_dial_min = 100;
  config.wrist_dial_max = 3000;

  bool result = elevated_control::ValidateButtonConfig(config);

  EXPECT_FALSE(result);
}

// -- ValidateSpringSetpoints tests --

TEST_F(ConfigParsingTest, ValidateSpringSetpointsValidConfig) {
  elevated_control::SpringSetpointsConfig config;
  config.spring_setpoint_unloaded = 200;
  config.spring_setpoint_loaded = 2000;

  bool result = elevated_control::ValidateSpringSetpoints(config);

  EXPECT_TRUE(result);
}

TEST_F(ConfigParsingTest, ValidateSpringSetpointsInvalidUnloadedTooLow) {
  elevated_control::SpringSetpointsConfig config;
  config.spring_setpoint_unloaded = 99;
  config.spring_setpoint_loaded = 2000;

  bool result = elevated_control::ValidateSpringSetpoints(config);

  EXPECT_FALSE(result);
}

TEST_F(ConfigParsingTest, ValidateSpringSetpointsInvalidUnloadedTooHigh) {
  elevated_control::SpringSetpointsConfig config;
  config.spring_setpoint_unloaded = 1001;
  config.spring_setpoint_loaded = 2000;

  bool result = elevated_control::ValidateSpringSetpoints(config);

  EXPECT_FALSE(result);
}

TEST_F(ConfigParsingTest, ValidateSpringSetpointsInvalidLoadedTooLow) {
  elevated_control::SpringSetpointsConfig config;
  config.spring_setpoint_unloaded = 200;
  config.spring_setpoint_loaded = 99;

  bool result = elevated_control::ValidateSpringSetpoints(config);

  EXPECT_FALSE(result);
}

TEST_F(ConfigParsingTest, ValidateSpringSetpointsInvalidLoadedTooHigh) {
  elevated_control::SpringSetpointsConfig config;
  config.spring_setpoint_unloaded = 200;
  config.spring_setpoint_loaded = 3901;

  bool result = elevated_control::ValidateSpringSetpoints(config);

  EXPECT_FALSE(result);
}

TEST_F(ConfigParsingTest, ValidateSpringSetpointsBoundaryValues) {
  elevated_control::SpringSetpointsConfig config;
  config.spring_setpoint_unloaded = 100;
  config.spring_setpoint_loaded = 3900;

  bool result = elevated_control::ValidateSpringSetpoints(config);

  EXPECT_TRUE(result);
}

// -- ValidateTorqueSignsConfig tests --

TEST_F(ConfigParsingTest, ValidateTorqueSignsConfigAllOnes) {
  elevated_control::TorqueSignConfig config;
  config.yaw1 = 1;
  config.yaw2 = 1;
  config.wrist_yaw = 1;

  bool result = elevated_control::ValidateTorqueSignsConfig(config);

  EXPECT_TRUE(result);
}

TEST_F(ConfigParsingTest, ValidateTorqueSignsConfigAllNegativeOnes) {
  elevated_control::TorqueSignConfig config;
  config.yaw1 = -1;
  config.yaw2 = -1;
  config.wrist_yaw = -1;

  bool result = elevated_control::ValidateTorqueSignsConfig(config);

  EXPECT_TRUE(result);
}

TEST_F(ConfigParsingTest, ValidateTorqueSignsConfigMixedValues) {
  elevated_control::TorqueSignConfig config;
  config.yaw1 = 1;
  config.yaw2 = -1;
  config.wrist_yaw = 1;

  bool result = elevated_control::ValidateTorqueSignsConfig(config);

  EXPECT_TRUE(result);
}

TEST_F(ConfigParsingTest, ValidateTorqueSignsConfigInvalidYaw1Zero) {
  elevated_control::TorqueSignConfig config;
  config.yaw1 = 0;
  config.yaw2 = 1;
  config.wrist_yaw = 1;

  bool result = elevated_control::ValidateTorqueSignsConfig(config);

  EXPECT_FALSE(result);
}

TEST_F(ConfigParsingTest, ValidateTorqueSignsConfigInvalidYaw1Two) {
  elevated_control::TorqueSignConfig config;
  config.yaw1 = 2;
  config.yaw2 = 1;
  config.wrist_yaw = 1;

  bool result = elevated_control::ValidateTorqueSignsConfig(config);

  EXPECT_FALSE(result);
}

TEST_F(ConfigParsingTest, ValidateTorqueSignsConfigInvalidYaw2Zero) {
  elevated_control::TorqueSignConfig config;
  config.yaw1 = 1;
  config.yaw2 = 0;
  config.wrist_yaw = 1;

  bool result = elevated_control::ValidateTorqueSignsConfig(config);

  EXPECT_FALSE(result);
}

TEST_F(ConfigParsingTest, ValidateTorqueSignsConfigInvalidYaw2Two) {
  elevated_control::TorqueSignConfig config;
  config.yaw1 = 1;
  config.yaw2 = 2;
  config.wrist_yaw = 1;

  bool result = elevated_control::ValidateTorqueSignsConfig(config);

  EXPECT_FALSE(result);
}

TEST_F(ConfigParsingTest, ValidateTorqueSignsConfigInvalidWristYawZero) {
  elevated_control::TorqueSignConfig config;
  config.yaw1 = 1;
  config.yaw2 = 1;
  config.wrist_yaw = 0;

  bool result = elevated_control::ValidateTorqueSignsConfig(config);

  EXPECT_FALSE(result);
}

TEST_F(ConfigParsingTest, ValidateTorqueSignsConfigInvalidWristYawTwo) {
  elevated_control::TorqueSignConfig config;
  config.yaw1 = 1;
  config.yaw2 = 1;
  config.wrist_yaw = 2;

  bool result = elevated_control::ValidateTorqueSignsConfig(config);

  EXPECT_FALSE(result);
}
