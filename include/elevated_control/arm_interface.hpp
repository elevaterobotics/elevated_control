// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <expected>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <spdlog/spdlog.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "ethercat.h"
#pragma GCC diagnostic pop

#include "elevated_control/config_parsing.hpp"
#include "elevated_control/constants.hpp"
#include "elevated_control/dynamic_sim.hpp"
#include "elevated_control/joint_admittance.hpp"
#include "elevated_control/somanet_pdo.hpp"
#include "elevated_control/types.hpp"
#include "elevated_control/velocity_filter.hpp"

namespace elevated_control {

struct SpringAdjustState {
  std::chrono::steady_clock::time_point time_prev;
  std::optional<float> error_prev;
};

// Joint names in EtherCAT bus order, used for YAML config lookup
inline const std::vector<std::string> kJointNames = {
    "yaw_1_joint",          "yaw_2_joint",
    "spring_adjust_joint",  "elevation_inertial_joint",
    "wrist_yaw_joint",      "wrist_pitch_joint",
    "wrist_roll_joint",
};

struct JointLimitsInfo {
  float min_position;
  float max_position;
  bool has_limits;
};

class ArmInterface {
 public:
  struct Config {
    std::string network_interface = "eno0";
    std::string joint_limits_yaml;
    std::string elevate_config_yaml = "/home/elevate/Desktop/elevate_config.yaml";
  };

  explicit ArmInterface(const Config& config);
  ~ArmInterface();

  ArmInterface(const ArmInterface&) = delete;
  ArmInterface& operator=(const ArmInterface&) = delete;

  // -- Lifecycle --

  std::expected<void, Error> Initialize();
  std::expected<void, Error> StartControlLoop(float control_rate_hz);
  std::expected<void, Error> StopControlLoop();

  // -- Control mode --

  std::expected<void, Error> SwitchControlMode(ControlLevel mode);
  std::expected<void, Error> SwitchControlMode(
      const std::vector<ControlLevel>& per_joint_modes);

  // -- Streaming commands --
  // These also switch the control mode accordingly.

  std::expected<void, Error> SetPositionCommand(
      const std::vector<float>& positions,
      std::function<bool()> halt_condition = nullptr);
  std::expected<void, Error> SetVelocityCommand(
      const std::vector<float>& velocities,
      std::function<bool()> halt_condition = nullptr);
  std::expected<void, Error> SetTorqueCommand(
      const std::vector<float>& torques,
      std::function<bool()> halt_condition = nullptr);

  // -- Generic command (per current mode) --

  std::expected<void, Error> SendCommand(
      const std::vector<float>& joint_commands);
  std::expected<void, Error> SendCommand(
      const std::vector<JointName>& joints,
      const std::vector<float>& joint_commands);

  // -- Trajectory --

  std::expected<void, Error> SetTrajectory(
      const std::vector<float>& positions,
      const std::vector<float>& time_from_start);

  // -- Spring --

  std::expected<void, Error> SetSpringSetpoint(float load_in_newtons);

  // -- State queries --

  std::expected<std::vector<float>, Error> GetPositions() const;
  std::expected<std::vector<float>, Error> GetVelocities() const;
  std::expected<std::vector<float>, Error> GetTorques() const;
  std::expected<std::vector<float>, Error> GetAccelerations() const;

  // -- GPIO --

  // Returns {function_enable, comp_button, decomp_button}
  std::expected<std::vector<bool>, Error> GetButtonState() const;
  // Returns {wrist_pitch_dial, wrist_roll_dial} (raw values)
  std::expected<std::vector<int>, Error> GetDialState() const;

  // -- Joint limits --

  std::expected<std::vector<JointLimitsInfo>, Error> GetJointLimits() const;

 private:
  // EtherCAT cyclic control loop (runs in control_thread_)
  void ControlLoop(std::stop_token stop_token);

  // Per-joint Somanet state machine step
  void StateMachineStep(std::size_t joint_idx,
                        std::int32_t lips_spring_position,
                        bool admittance_button_pressed);

  void StatusWordErrorLogging(std::size_t joint_idx);

  // Apply gravity compensation from the dynamic simulation
  void ApplyGravComp(std::size_t joint_idx, const InSomanet50t* in_somanet,
                     OutSomanet50t* out_somanet);

  void ApplyWristPitchHoldTorque(const InSomanet50t* in_somanet,
                                 OutSomanet50t* out_somanet);

  void ApplyFrictionCompensation(std::size_t joint_idx);

  float CalculateUserTorque(const InSomanet50t* in_somanet,
                            std::size_t joint_idx);

  // Spring adjust PD control via linear potentiometer
  float SpringAdjustByLIPS(float target_position,
                           std::int32_t current_lips_position,
                           bool& allow_mode_change);

  // E-stop and process data check
  bool EStopEngaged();

  // EtherCAT error recovery (runs periodically)
  void EcatCheck(std::stop_token stop_token);

  // Wait for initial process data communication
  void WaitForGoodProcessData();

  // Attempt startup fault recovery
  void HandleStartupFaults();

  // Reset spring adjust PD state
  void ResetSpringAdjustState();

  // Internal mode switch implementation
  std::expected<void, Error> SwitchControlModeImpl(
      const std::vector<ControlLevel>& new_modes);

  // Config
  Config config_;

  // State
  std::atomic<bool> initialized_{false};
  std::atomic<bool> control_loop_running_{false};
  std::atomic<bool> in_normal_op_mode_{false};
  std::atomic<bool> require_new_command_mode_{false};
  std::atomic<bool> mode_switch_in_progress_{false};
  std::atomic<bool> allow_mode_change_{true};
  std::atomic<bool> dynamic_sim_exited_{false};

  // Threads
  std::jthread control_thread_;
  std::jthread dynamic_sim_thread_;
  std::jthread ecat_error_thread_;

  // EtherCAT
  char io_map_[4096]{};
  std::vector<InSomanet50t*> in_somanet_;
  std::vector<OutSomanet50t*> out_somanet_;
  std::atomic<int> wkc_{0};
  std::atomic<int> expected_wkc_{0};

  // Per-joint mechanical params
  std::deque<std::atomic<float>> mechanical_reductions_;
  std::deque<std::atomic<std::uint32_t>> encoder_resolutions_;
  std::deque<std::atomic<float>> rated_torques_;

  // Thread-safe command buffers
  std::deque<std::atomic<float>> threadsafe_commands_positions_;
  std::deque<std::atomic<float>> threadsafe_commands_velocities_;
  std::deque<std::atomic<float>> threadsafe_commands_efforts_;
  std::deque<std::atomic<float>> threadsafe_commands_spring_adjust_;

  // Halt condition (set by Set*Command, checked in control loop)
  std::mutex halt_mutex_;
  std::function<bool()> halt_condition_;

  // Per-joint control modes
  std::vector<ControlLevel> control_level_;
  mutable std::mutex hw_state_mtx_;

  // State readout (updated from control loop under mutex)
  std::vector<float> state_positions_;
  std::vector<float> state_velocities_;
  std::vector<float> state_torques_;
  std::vector<float> state_accelerations_;
  std::atomic<float> hw_function_enable_{0.0f};
  std::atomic<float> hw_comp_button_{0.0f};
  std::atomic<float> hw_decomp_button_{0.0f};

  // Dynamic simulation
  std::shared_ptr<DynamicSimState> dynamic_sim_state_;
  DynamicSimulator dynamic_simulator_;

  // Joint limits
  std::vector<bool> has_position_limits_;
  std::vector<float> min_position_limits_;
  std::vector<float> max_position_limits_;

  // Config from YAML
  ElevateConfig elevate_config_;

  // Spring adjust
  SpringAdjustState spring_adjust_state_;
  std::atomic<float> spring_setpoint_target_{0.0f};

  // Admittance
  std::vector<std::optional<JointAdmittance>> joint_admittances_;

  // Filters
  VelocityFilter wrist_pitch_dial_filter_{0.3f};
  VelocityFilter yaw1_torque_filter_{0.92f};
  VelocityFilter yaw2_torque_filter_{0.7f};
  VelocityFilter wrist_yaw_torque_filter_{0.7f};
  VelocityFilter elevation_inertial_torque_filter_{0.7f};
  VelocityFilter yaw1_collision_torque_filter_{0.7f};
  VelocityFilter yaw2_collision_torque_filter_{0.7f};
  VelocityFilter wrist_yaw_collision_torque_filter_{0.7f};

  // Hand-guided wrist pitch brake hysteresis
  std::atomic<bool> hand_guided_pitch_brake_state_{false};
};

}  // namespace elevated_control
