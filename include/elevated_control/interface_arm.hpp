// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <spdlog/spdlog.h>

#include "elevated_control/constants_arm.hpp"
#include "elevated_control/types_arm.hpp"
#include "elevated_control/config_parsing.hpp"
#include "elevated_control/dial_normalization.hpp"
#include "elevated_control/dynamic_sim.hpp"
#include "elevated_control/joint_admittance.hpp"
#include "elevated_control/spring_adjust.hpp"
#include "elevated_control/synapticcon_base.hpp"
#include "elevated_control/velocity_filter.hpp"

namespace elevated_control {

class ArmInterface : public SynapticonBase {
 public:
  struct Config : SynapticonBaseConfig {
    std::string elevate_config_yaml = "/home/elevate/Desktop/elevate_config.yaml";
    Config() { expected_slave_count = kNumJoints; }
  };

  explicit ArmInterface(const Config& config);
  ~ArmInterface() override;

  ArmInterface(const ArmInterface&) = delete;
  ArmInterface& operator=(const ArmInterface&) = delete;

  // -- Lifecycle (override base) --

  std::expected<void, Error> Initialize() override;
  std::expected<void, Error> StartControlLoop(float control_rate_hz) override;
  std::expected<void, Error> StopControlLoop() override;

  // -- Control mode (arm-specific overloads) --

  std::expected<void, Error> SwitchControlMode(ControlLevel mode);
  std::expected<void, Error> SwitchControlMode(
      const JointControlLevelArray& per_joint_modes);

  // Bring base overloads into scope
  using SynapticonBase::SwitchControlMode;

  // -- Streaming commands (JointFloatArray overloads) --

  std::expected<void, Error> SetPositionCommand(
      const JointFloatArray& positions,
      std::function<bool()> halt_condition = nullptr);
  std::expected<void, Error> SetVelocityCommand(
      const JointFloatArray& velocities,
      std::function<bool()> halt_condition = nullptr);
  std::expected<void, Error> SetTorqueCommand(
      const JointFloatArray& torques,
      std::function<bool()> halt_condition = nullptr);

  // Bring base overloads into scope
  using SynapticonBase::SetPositionCommand;
  using SynapticonBase::SetVelocityCommand;
  using SynapticonBase::SetTorqueCommand;

  // -- Generic command (per current mode) --

  std::expected<void, Error> SendCommand(
      const JointFloatArray& joint_commands);
  std::expected<void, Error> SendCommand(
      const std::vector<JointName>& joints,
      const std::vector<float>& joint_commands);

  using SynapticonBase::SendCommand;

  // -- Spring --

  std::expected<void, Error> SetSpringSetpoint(float load_in_newtons);

  // -- State queries (JointFloatArray wrappers) --

  std::expected<JointFloatArray, Error> GetPositionsArray() const;
  std::expected<JointFloatArray, Error> GetVelocitiesArray() const;
  std::expected<JointFloatArray, Error> GetTorquesArray() const;
  std::expected<JointFloatArray, Error> GetAccelerationsArray() const;

  // -- GPIO --

  std::expected<std::vector<bool>, Error> GetButtonState() const;
  std::expected<std::vector<int>, Error> GetDialState() const;

  // -- Joint limits --

  std::expected<JointArray<JointLimitsInfo>, Error> GetJointLimitsArray() const;

 protected:
  // Override virtual hooks
  void OnPreStateMachine() override;
  bool OnNormalOperation(std::size_t joint_idx, bool mode_switch) override;
  bool IsEStopEngaged() override;

 private:
  std::expected<void, Error> SwitchControlModeImpl(
      const JointControlLevelArray& new_modes);

  void ApplyGravComp(std::size_t joint_idx, const InSomanet50t* in_somanet,
                     OutSomanet50t* out_somanet);
  void ApplyWristPitchHoldTorque(const InSomanet50t* in_somanet,
                                 OutSomanet50t* out_somanet);
  void RefreshHandGuidedPitchBrakeHold();
  void ApplyFrictionCompensation(std::size_t joint_idx);
  float CalculateUserTorque(const InSomanet50t* in_somanet,
                            std::size_t joint_idx);
  void ResetSpringAdjustState();

  // Arm config
  Config arm_config_;

  // Dynamic simulation
  std::shared_ptr<DynamicSimState> dynamic_sim_state_;
  DynamicSimulator dynamic_simulator_;
  std::jthread dynamic_sim_thread_;
  std::atomic<bool> dynamic_sim_exited_{false};

  // Per-joint control modes (arm-specific superset)
  JointControlLevelArray control_level_{};

  // Config from YAML
  ElevateConfig elevate_config_;

  // Spring adjust
  SpringAdjustState spring_adjust_state_;
  std::atomic<float> spring_setpoint_target_ticks_{0.0f};
  std::atomic<bool> has_spring_setpoint_{false};

  // Admittance
  JointArray<std::optional<JointAdmittance>> joint_admittances_{};

  // Filters
  VelocityFilter wrist_pitch_dial_filter_{0.9f};
  DialActivationState wrist_pitch_dial_state_{};
  VelocityFilter wrist_roll_dial_filter_{0.9f};
  VelocityFilter wrist_roll_admittance_filter_{0.98f};
  VelocityFilter yaw1_torque_filter_{0.92f};
  VelocityFilter yaw2_torque_filter_{0.7f};
  VelocityFilter wrist_yaw_torque_filter_{0.7f};
  VelocityFilter elevation_inertial_torque_filter_{0.7f};
  VelocityFilter yaw1_collision_torque_filter_{0.7f};
  VelocityFilter yaw2_collision_torque_filter_{0.7f};
  VelocityFilter wrist_yaw_collision_torque_filter_{0.7f};

  // GPIO state
  std::atomic<float> hw_function_enable_{0.0f};
  std::atomic<float> hw_comp_button_{0.0f};
  std::atomic<float> hw_decomp_button_{0.0f};

  // Pre-state-machine state
  bool first_iteration_deadman_{true};
  bool prev_deadman_pressed_{false};
  std::int32_t lips_spring_position_{0};
  bool admittance_button_pressed_{false};
  bool deadman_pressed_{false};
  bool skip_cycle_{false};
};

}  // namespace elevated_control
