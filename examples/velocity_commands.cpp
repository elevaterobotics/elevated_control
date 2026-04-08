#include "elevated_control/arm_interface.hpp"
#include "example_config_dir.hpp"

#include <chrono>
#include <cstdlib>
#include <limits>
#include <thread>

#include "elevated_control/arm_types.hpp"

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

  // Read initial joint positions
  auto pos = arm.GetPositions();
  if (pos) {
    for (float p : *pos) spdlog::info("{:.4f}", p);
  }

  // Send a slow velocity command to the wrist roll joint
  elevated_control::JointFloatArray velocities{};
  // spring_adjust_joint (index 2) must be NaN for SetVelocityCommand()
  // Use SetSpringSetpoint() instead if you want to adjust the spring load.
  velocities[elevated_control::JointIndex(
      elevated_control::JointName::kSpringAdjust)] =
      std::numeric_limits<float>::quiet_NaN();
  // Wrist roll command
  velocities[elevated_control::JointIndex(
      elevated_control::JointName::kWristRoll)] = 0.02f;

  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < end) {
    auto cmd = arm.SetVelocityCommand(velocities);
    if (!cmd) {
      spdlog::error("SetVelocityCommand failed: {}", cmd.error().message);
      arm.StopControlLoop();
      return EXIT_FAILURE;
    }
    // We need to send a new command faster than 200ms or the driver will timeout and stop the joint
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Read final joint positions
  pos = arm.GetPositions();
  if (pos) {
    for (float p : *pos) spdlog::info("{:.4f}", p);
  }

  arm.StopControlLoop();
  return EXIT_SUCCESS;
}
