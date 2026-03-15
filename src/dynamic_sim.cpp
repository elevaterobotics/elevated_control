// Copyright (c) 2025 Elevate Robotics Inc

#include "elevated_control/dynamic_sim.hpp"

#include <condition_variable>
#include <mutex>

#include <spdlog/spdlog.h>

namespace elevated_control {

bool DynamicSimState::SetJointPositions(
    const std::vector<std::pair<std::string, float>>& /*joint_name_to_position*/) {
  // Stub: name-based position setting not yet implemented (needs Drake indices)
  return false;
}

void InitializeDynamicSimState(std::shared_ptr<DynamicSimState> state,
                               std::size_t num_state_entries) {
  state->input.positions.resize(num_state_entries);
  state->input.velocities.resize(num_state_entries);
  state->output.estimated_accelerations.resize(num_state_entries);
  state->output.tau_required.resize(num_state_entries);
  state->output.joints_to_halt.resize(num_state_entries);

  for (std::size_t i = 0; i < num_state_entries; ++i) {
    state->input.positions[i] = 0.0f;
    state->input.velocities[i] = 0.0f;
    state->output.estimated_accelerations[i] = 0.0f;
    state->output.tau_required[i] = 0.0f;
    state->output.joints_to_halt[i] = false;
  }

  state->input.payload_mass = 0.0f;
  state->input.base_link_pitch = 0.0f;
  state->input.base_link_roll = 0.0f;
  state->input.base_link_yaw = 0.0f;
}

DynamicSimulator::DynamicSimulator(
    std::shared_ptr<DynamicSimState> shared_state)
    : dynamic_sim_state_(std::move(shared_state)) {}

bool DynamicSimulator::Run(std::stop_token stop_token) {
  spdlog::info("DynamicSimulator: stub started (no-op until Drake is wired)");

  // Block until cancellation is requested
  std::mutex mtx;
  std::condition_variable_any cv;
  std::unique_lock lock(mtx);
  cv.wait(lock, stop_token, [] { return false; });

  spdlog::info("DynamicSimulator: stop requested, exiting");
  return true;
}

}  // namespace elevated_control
