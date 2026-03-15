// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <atomic>
#include <deque>
#include <memory>
#include <stop_token>
#include <string>
#include <utility>
#include <vector>

#include "elevated_control/types.hpp"

namespace elevated_control {

// Shared state between the EtherCAT control loop and the dynamic simulation
// thread. All fields are atomic for lock-free access.
struct DynamicSimState {
  struct SimInput {
    std::deque<std::atomic<float>> positions;
    std::deque<std::atomic<float>> velocities;
    std::atomic<float> payload_mass{0.0f};
    std::atomic<float> base_link_pitch{0.0f};
    std::atomic<float> base_link_roll{0.0f};
    std::atomic<float> base_link_yaw{0.0f};
  };

  struct SimOutput {
    std::deque<std::atomic<float>> estimated_accelerations;
    // Torque required to hold the robot against gravity (feedforward)
    std::deque<std::atomic<float>> tau_required;
    // Joints that should be halted due to collision proximity
    std::deque<std::atomic<bool>> joints_to_halt;
  };

  SimInput input;
  SimOutput output;

  bool SetJointPositions(
      const std::vector<std::pair<std::string, float>>& joint_name_to_position);
};

// Initialize a DynamicSimState with zero positions/velocities and
// identity base orientation.
void InitializeDynamicSimState(std::shared_ptr<DynamicSimState> state,
                               std::size_t num_state_entries);

// Stub dynamic simulator. Currently a no-op; Run() blocks on the stop_token
// until cancellation is requested. tau_required remains zero, so gravity
// compensation is effectively disabled.
class DynamicSimulator {
 public:
  explicit DynamicSimulator(std::shared_ptr<DynamicSimState> shared_state);

  // Blocks until stop is requested. Returns true on clean exit.
  bool Run(std::stop_token stop_token);

 private:
  std::shared_ptr<DynamicSimState> dynamic_sim_state_;
};

}  // namespace elevated_control
