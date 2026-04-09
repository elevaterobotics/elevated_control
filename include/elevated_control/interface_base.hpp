// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <expected>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <spdlog/spdlog.h>

#include "elevated_control/constants.hpp"
#include "elevated_control/somanet_pdo.hpp"
#include "elevated_control/types.hpp"
#include "elevated_control/velocity_filter.hpp"

namespace elevated_control {

// Position limits in the same units as GetPositions / SetPositionCommand (output-shaft rad).
struct JointLimitsInfo {
  float min_position;
  float max_position;
  bool has_limits;
};

// Thread-safe snapshot of PDO input fields.
struct InSomanetSnapshot {
  std::int32_t position_value = 0;
  std::int32_t velocity_value = 0;
  std::int16_t torque_value = 0;
};

struct SynapticonBaseConfig {
  std::string network_interface = "eno0";
  std::size_t expected_slave_count = 0;  // 0 = auto-detect, >0 = verify
  std::string joint_limits_yaml;
};

// EtherCAT / CiA 402 driver interface. All joint-space quantities are on the output shaft
// (after gearbox), unless a derived class documents otherwise.
//
// Units:
//   Position commands / feedback, and JointLimitsInfo: radians (rad).
//   Velocity commands / feedback: rad/s.
//   Torque commands: per mille (‰) of rated drive torque, clamped to about [-1000, 1000],
//     written as CiA 402 Target Torque. Torque feedback is Nm at the output shaft.
//   Acceleration feedback: currently always 0; would be rad/s² if implemented.
//   StartControlLoop: control_rate_hz is Hertz (Hz).
//
// SetTrajectory is not implemented; no unit contract until it is.
class SynapticonBase {
 public:
  explicit SynapticonBase(const SynapticonBaseConfig& config);
  virtual ~SynapticonBase();

  SynapticonBase(const SynapticonBase&) = delete;
  SynapticonBase& operator=(const SynapticonBase&) = delete;

  // -- Lifecycle --

  virtual std::expected<void, Error> Initialize();
  // control_rate_hz: cyclic EtherCAT / control thread rate [Hz].
  virtual std::expected<void, Error> StartControlLoop(float control_rate_hz);
  virtual std::expected<void, Error> StopControlLoop();
  bool IsControlLoopReady() const noexcept;

  template <typename Rep, typename Period>
  bool WaitForLoopReady(
      std::chrono::duration<Rep, Period> wait_time) const {
    const auto deadline = std::chrono::steady_clock::now() + wait_time;
    while (!IsControlLoopReady()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
  }

  // -- Control mode --

  virtual std::expected<void, Error> SwitchControlMode(ControlMode mode);
  virtual std::expected<void, Error> SwitchControlMode(
      const std::vector<ControlMode>& per_joint_modes);

  // -- Streaming commands --
  // positions: output-shaft rad per joint.
  virtual std::expected<void, Error> SetPositionCommand(
      const std::vector<float>& positions,
      std::function<bool()> halt_condition = nullptr);
  // velocities: output-shaft rad/s per joint.
  virtual std::expected<void, Error> SetVelocityCommand(
      const std::vector<float>& velocities,
      std::function<bool()> halt_condition = nullptr);
  // torques: per mille (‰) of rated torque per joint
  virtual std::expected<void, Error> SetTorqueCommand(
      const std::vector<float>& torques,
      std::function<bool()> halt_condition = nullptr);

  // -- Generic command (per current mode) --
  // joint_commands: same units as SetPositionCommand / SetVelocityCommand / SetTorqueCommand for
  // kPosition / kVelocity / kTorque respectively.
  virtual std::expected<void, Error> SendCommand(
      const std::vector<float>& joint_commands);

  // -- Trajectory --
  // Not implemented; parameters reserved for future use.

  std::expected<void, Error> SetTrajectory(
      const std::vector<float>& positions,
      const std::vector<float>& time_from_start);

  // -- State queries --
  // Positions: rad; velocities: rad/s; torques: Nm; accelerations: placeholder (0).
  std::expected<std::vector<float>, Error> GetPositions() const;
  std::expected<std::vector<float>, Error> GetVelocities() const;
  std::expected<std::vector<float>, Error> GetTorques() const;
  std::expected<std::vector<float>, Error> GetAccelerations() const;

  // -- Joint info --

  std::expected<std::vector<JointLimitsInfo>, Error> GetJointLimits() const;
  std::size_t num_joints() const noexcept { return num_joints_; }

 protected:
  // Virtual hooks for derived classes.
  // Called once per control cycle before the per-joint state machine.
  virtual void OnPreStateMachine() {}
  // Called for each joint during normal operation. Return true if handled.
  virtual bool OnNormalOperation(std::size_t joint_idx, bool mode_switch) {
    (void)joint_idx; (void)mode_switch;
    return false;
  }
  virtual void OnPostCycle() {}
  // E-stop check; base implementation checks WKC only.
  virtual bool IsEStopEngaged();

  // EtherCAT cyclic control loop
  void ControlLoop(std::stop_token stop_token);
  void UpdateInSomanetSnapshotLocked();
  void BaseStateMachineStep(std::size_t joint_idx, bool mode_switch);
  void StatusWordErrorLogging(std::size_t joint_idx);

  void WaitForGoodProcessData();
  void HandleStartupFaults();

  void EcatCheck(std::stop_token stop_token);

  std::expected<void, Error> SwitchControlModeImpl(
      const std::vector<ControlMode>& new_modes);

  // Config
  SynapticonBaseConfig base_config_;
  std::size_t num_joints_{0};

  // State
  std::atomic<bool> initialized_{false};
  std::atomic<bool> control_loop_running_{false};
  std::atomic<bool> in_normal_op_mode_{false};
  std::atomic<bool> require_new_command_mode_{false};
  std::atomic<bool> mode_switch_in_progress_{false};
  std::atomic<bool> allow_mode_change_{true};
  std::atomic<bool> shutdown_requested_{false};

  // Threads
  std::jthread control_thread_;
  std::jthread ecat_error_thread_;

  // EtherCAT
  char io_map_[4096]{};
  std::mutex ecat_mtx_;
  std::vector<InSomanet50t*> in_somanet_;
  std::vector<InSomanetSnapshot> in_somanet_snapshot_;
  std::vector<OutSomanet50t*> out_somanet_;
  std::atomic<int> wkc_{0};
  std::atomic<int> expected_wkc_{0};
  std::atomic<int> pdo_exchange_count_{0};

  // Per-joint mechanical params
  std::deque<std::atomic<float>> mechanical_reductions_;
  std::deque<std::atomic<float>> position_reductions_;
  std::deque<std::atomic<std::int32_t>> si_velocity_units_;
  std::deque<std::atomic<std::uint32_t>> encoder_resolutions_;
  std::deque<std::atomic<float>> rated_torques_;

  // Startup angle wrap state (replaces global arrays)
  std::deque<std::once_flag> startup_angle_wrap_flag_;
  std::deque<std::atomic<float>> startup_angle_wrap_value_;

  // Thread-safe command buffers
  std::deque<std::atomic<float>> threadsafe_commands_positions_;
  std::deque<std::atomic<float>> threadsafe_commands_velocities_;
  std::deque<std::atomic<int64_t>> last_velocity_write_time_ns_;
  static constexpr int64_t VELOCITY_COMMAND_TIMEOUT_NS = 200'000'000;
  std::deque<std::atomic<float>> threadsafe_commands_efforts_;

  // Halt condition
  std::mutex halt_mutex_;
  std::function<bool()> halt_condition_;

  // Per-joint control modes (base modes only)
  std::vector<ControlMode> control_mode_;
  std::deque<std::atomic<bool>> hold_in_shutdown_;
  mutable std::mutex hw_state_mtx_;

  // State readout
  std::vector<float> state_positions_;
  std::vector<float> state_velocities_;
  std::vector<float> state_torques_;
  std::vector<float> state_accelerations_;

  // Joint limits
  std::vector<bool> has_position_limits_;
  std::vector<float> min_position_limits_;
  std::vector<float> max_position_limits_;
  std::vector<float> max_brake_torques_;

  // Per-joint velocity filters (one per joint, for base velocity limit handling)
  std::vector<VelocityFilter> velocity_filters_;
};

}  // namespace elevated_control
