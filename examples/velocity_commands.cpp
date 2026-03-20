#include "elevated_control/arm_interface.hpp"
#include "example_config_dir.hpp"

#include <chrono>
#include <thread>
#include <vector>

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
    return 1;
  }

  auto start = arm.StartControlLoop(200.0f);  // 200 Hz
  if (!start) {
    spdlog::error("Start failed: {}", start.error().message);
    return 1;
  }

  while (!arm.IsControlLoopReady()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Output-shaft rad/s; wrist roll joint only, small rate. Re-issue before 200 ms
  // timeout in the control loop so velocity is held for the full window.
  std::vector<float> velocities(elevated_control::kNumJoints, 0.0f);
  velocities[6] = 0.02f;

  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < end) {
    auto cmd = arm.SetVelocityCommand(velocities);
    if (!cmd) {
      spdlog::error("SetVelocityCommand failed: {}", cmd.error().message);
      arm.StopControlLoop();
      return 1;
    }
    // We need to send a new command faster than 200ms or the driver will timeout and stop the joint
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  arm.StopControlLoop();
}
