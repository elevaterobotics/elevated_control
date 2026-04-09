#include "elevated_control/interface_single_joint.hpp"

#include <chrono>
#include <cstdlib>
#include <thread>

int main() {
  spdlog::set_level(spdlog::level::debug);

  elevated_control::SingleJointInterface::Config cfg;
  cfg.network_interface = "eno0";

  elevated_control::SingleJointInterface motor(cfg);

  auto init = motor.Initialize();
  if (!init) {
    spdlog::error("Init failed: {}", init.error().message);
    return EXIT_FAILURE;
  }

  auto start = motor.StartControlLoop(200.0f /* Hz */);
  if (!start) {
    spdlog::error("Start failed: {}", start.error().message);
    return EXIT_FAILURE;
  }

  if (!motor.WaitForLoopReady(std::chrono::seconds(10))) {
    spdlog::error("Control loop did not become ready in time");
    motor.StopControlLoop();
    return EXIT_FAILURE;
  }

  // Read initial position
  auto pos = motor.GetPosition();
  float initial_pos = 0.0f;
  if (pos) {
    initial_pos = *pos;
    spdlog::info("Initial position: {:.6f} rad", *pos);
  }

  // Send a slow velocity command for 3 seconds
  spdlog::info("Commanding velocity 0.1 rad/s for 3 seconds (expected delta: 0.3 rad)");
  motor.SwitchControlMode(elevated_control::ControlMode::kVelocity);
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < end) {
    auto cmd = motor.SetVelocity(0.1f);
    if (!cmd) {
      spdlog::error("SetVelocity failed: {}", cmd.error().message);
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Read final position
  pos = motor.GetPosition();
  if (pos) {
    float delta = *pos - initial_pos;
    spdlog::info("Final position: {:.6f} rad", *pos);
    spdlog::info("Position delta: {:.6f} rad (expected ~0.3 rad)", delta);
    spdlog::info("Ratio actual/expected: {:.2f}x", delta / 0.3f);
  }

  motor.StopControlLoop();
  return EXIT_SUCCESS;
}
