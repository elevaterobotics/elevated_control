#include "elevated_control/arm_interface.hpp"

int main() {
  elevated_control::ArmInterface::Config cfg;
  cfg.network_interface = "eno0";
  cfg.joint_limits_yaml = "joint_limits.yaml";
  cfg.elevate_config_yaml = "elevate_config.yaml";

  elevated_control::ArmInterface arm(cfg);

  auto init = arm.Initialize();
  if (!init) {
    spdlog::error("Init failed: {}", init.error().message);
    return 1;
  }

  auto start = arm.StartControlLoop(200.0f);  // 200 Hz
  if (!start) {
    spdlog::error("Start failed: {}", start.error().message);
    return 1;
  }

  // Read joint positions
  auto pos = arm.GetPositions();
  if (pos) {
    for (float p : *pos) spdlog::info("{:.4f}", p);
  }

  // Send a position command (7 joints, radians)
  //std::vector<float> target(7, 0.0f);
  //arm.SetPositionCommand(target);

  // Switch to hand-guided mode
  //arm.SwitchControlMode(elevated_control::ControlLevel::kHandGuided);

  // Stop and apply brakes
  arm.StopControlLoop();
}
