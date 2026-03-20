#include "elevated_control/arm_interface.hpp"

#include <filesystem>
#include <string>

namespace fs = std::filesystem;

// Config paths are relative to the current working directory. Resolve the
// directory that holds this executable (build tree or install prefix) so
// `./basic_usage` works from any cwd once YAML is copied/installed there.
static fs::path ExampleConfigDir() {
  std::error_code ec;
  const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
  if (!ec) {
    const fs::path dir = fs::weakly_canonical(exe).parent_path();
    if (fs::exists(dir / "joint_limits.yaml")) return dir;
    const fs::path share = dir.parent_path() / "share" / "elevated_control";
    if (fs::exists(share / "joint_limits.yaml")) return share;
    return dir;
  }
  return fs::current_path();
}

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

  std::this_thread::sleep_for(std::chrono::seconds(10));

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
