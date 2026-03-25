#include "elevated_control/arm_interface.hpp"
#include "example_config_dir.hpp"

#include <chrono>
#include <cstdlib>
#include <thread>

#include "elevated_control/types.hpp"

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

  // SetSpringSetpoint must be called before SwitchControlMode(kSpringAdjust)
  auto sp = arm.SetSpringSetpoint(0.0f /* load in Newtons */);
  if (!sp) {
    spdlog::error("SetSpringSetpoint failed: {}", sp.error().message);
    arm.StopControlLoop();
    return EXIT_FAILURE;
  }

  auto mode = arm.SwitchControlMode(elevated_control::ControlLevel::kSpringAdjust);
  if (!mode) {
    spdlog::error("SwitchControlMode(kSpringAdjust) failed: {}",
                  mode.error().message);
    arm.StopControlLoop();
    return EXIT_FAILURE;
  }

  spdlog::info("Spring adjust mode active; holding for 5s");
  std::this_thread::sleep_for(std::chrono::seconds(5));

  arm.StopControlLoop();
  return EXIT_SUCCESS;
}
