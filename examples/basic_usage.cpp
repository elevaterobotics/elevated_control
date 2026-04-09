#include "elevated_control/interface_arm.hpp"
#include "example_config_dir.hpp"

#include <chrono>
#include <cstdlib>

int main() {
  const fs::path dir = ExampleConfigDir();

  elevated_control::ArmInterface::Config cfg;
  cfg.network_interface = "eno0";
  cfg.joint_limits_yaml = (dir / "joint_limits.yaml").string();
  cfg.elevate_config_yaml = (dir / "elevate_config.yaml").string();

  elevated_control::ArmInterface arm(cfg);

  auto init = arm.Initialize();
  if (!init) {
    spdlog::error("Init failed: {}", init.error().message);
    return EXIT_FAILURE;
  }

  auto start = arm.StartControlLoop(200.0f);  // 200 Hz
  if (!start) {
    spdlog::error("Start failed: {}", start.error().message);
    return EXIT_FAILURE;
  }

  if (!arm.WaitForLoopReady(std::chrono::seconds(10))) {
    spdlog::error("Control loop did not become ready in time");
    arm.StopControlLoop();
    return EXIT_FAILURE;
  }

  // Read joint positions
  auto pos = arm.GetPositions();
  if (pos) {
    for (float p : *pos) spdlog::info("{:.4f}", p);
  }

  // Stop and apply brakes
  arm.StopControlLoop();
  return EXIT_SUCCESS;
}
