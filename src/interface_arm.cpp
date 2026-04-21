// Copyright (c) 2025 Elevate Robotics Inc

#include "elevated_control/interface_arm.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <soem/ethercat.h>
#pragma GCC diagnostic pop

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>

#include "elevated_control/joint_limits.hpp"
#include "elevated_control/state_machine.hpp"
#include "elevated_control/unit_conversions.hpp"

using namespace std::chrono_literals;

namespace elevated_control {

namespace {

JointBoolArray HasPositionLimitsToArray(const std::vector<bool>& v) {
  JointBoolArray arr{};
  for (std::size_t i = 0; i < kNumJoints && i < v.size(); ++i) arr[i] = v[i];
  return arr;
}

JointControlLevelArray MakeStreamingModes(ControlLevel mode) {
  JointControlLevelArray m;
  m.fill(mode);
  m[kSpringAdjustIdx] = ControlLevel::kSpringAdjust;
  return m;
}

}  // namespace

// ============================================================================
// Construction / Destruction
// ============================================================================

ArmInterface::ArmInterface(const Config& config)
    : SynapticonBase(config),
      arm_config_(config),
      dynamic_sim_state_(std::make_shared<DynamicSimState>()),
      dynamic_simulator_(dynamic_sim_state_) {}

ArmInterface::~ArmInterface() {
  if (dynamic_sim_thread_.joinable()) {
    dynamic_sim_thread_.request_stop();
    dynamic_sim_thread_.join();
  }
}

// ============================================================================
// Lifecycle
// ============================================================================

std::expected<void, Error> ArmInterface::Initialize() {
  // Base EtherCAT initialization
  auto result = SynapticonBase::Initialize();
  if (!result) return result;

  // Initialize arm-specific state
  control_level_.fill(ControlLevel::kUndefined);

  InitializeDynamicSimState(dynamic_sim_state_, kNumJoints);

  // Parse joint limits
  if (!base_config_.joint_limits_yaml.empty()) {
    auto joint_limits_config = ParseJointLimits(base_config_.joint_limits_yaml);
    if (!ValidateJointLimits(joint_limits_config)) {
      return std::unexpected(
          Error{ErrorCode::kInvalidArgument, "Invalid joint limits config"});
    }
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      has_position_limits_[i] = joint_limits_config.has_position_limits[i];
      min_position_limits_[i] = joint_limits_config.min_position_limits[i];
      max_position_limits_[i] = joint_limits_config.max_position_limits[i];
      max_brake_torques_[i] = kMaxJointLimitBrakeTorque[i];
    }
  }

  // Parse elevate config
  if (!arm_config_.elevate_config_yaml.empty()) {
    elevate_config_ = ParseElevateConfig(arm_config_.elevate_config_yaml);
    if (!ValidateButtonConfig(elevate_config_.button_config) ||
        !ValidateSpringSetpoints(elevate_config_.spring_setpoints) ||
        !ValidateTorqueSignsConfig(elevate_config_.torque_sign) ||
        !ValidatePositionSignsConfig(elevate_config_.position_sign) ||
        !ValidateVelocitySignsConfig(elevate_config_.velocity_sign)) {
      return std::unexpected(
          Error{ErrorCode::kInvalidArgument, "Invalid elevate config"});
    }
  }

  // Initialize joint admittances (wrist roll only)
  joint_admittances_[kWristRollIdx].emplace();
  if (!joint_admittances_[kWristRollIdx]->Init(10.0f, 500.0f)) {
    joint_admittances_[kWristRollIdx].reset();
    return std::unexpected(Error{ErrorCode::kEtherCATError,
                                 "Failed to init wrist roll admittance"});
  }

  spdlog::info("ArmInterface initialized successfully");
  return {};
}

std::expected<void, Error> ArmInterface::StartControlLoop(
    float control_rate_hz) {
  auto result = SynapticonBase::StartControlLoop(control_rate_hz);
  if (!result) return result;

  dynamic_sim_exited_ = false;
  dynamic_sim_thread_ = std::jthread([this](std::stop_token st) {
    bool ok = dynamic_simulator_.Run(st);
    if (!ok) {
      spdlog::error("Dynamic simulator exited with error");
    }
    dynamic_sim_exited_ = true;
  });

  return {};
}

std::expected<void, Error> ArmInterface::StopControlLoop() {
  dynamic_sim_thread_.request_stop();
  if (dynamic_sim_thread_.joinable()) dynamic_sim_thread_.join();

  // Apply brakes with arm control level before stopping base
  {
    std::scoped_lock lock(ecat_mtx_, hw_state_mtx_);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      control_level_[i] = ControlLevel::kUndefined;
    }
  }

  return SynapticonBase::StopControlLoop();
}

// ============================================================================
// Control mode switching
// ============================================================================

std::expected<void, Error> ArmInterface::SwitchControlMode(ControlLevel mode) {
  JointControlLevelArray modes;
  modes.fill(mode);
  return SwitchControlModeImpl(modes);
}

std::expected<void, Error> ArmInterface::SwitchControlMode(
    const JointControlLevelArray& per_joint_modes) {
  return SwitchControlModeImpl(per_joint_modes);
}

std::expected<void, Error> ArmInterface::SwitchControlModeImpl(
    const JointControlLevelArray& new_modes) {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  bool requested_quick_stop =
      std::any_of(new_modes.begin(), new_modes.end(),
                  [](ControlLevel m) { return m == ControlLevel::kQuickStop; });

  if (!prevent_mode_change_.AllowsNormalModeSwitch() && !requested_quick_stop) {
    return std::unexpected(
        Error{ErrorCode::kModeChangeDenied,
              "Control mode change is disallowed at this moment. Active blockers: " +
                  prevent_mode_change_.BlockingReasonsString()});
  }

  struct ModeSwitchGuard {
    std::atomic<bool>& flag;
    ModeSwitchGuard(std::atomic<bool>& f) : flag(f) { flag = true; }
    ~ModeSwitchGuard() { flag = false; }
  };
  ModeSwitchGuard guard(mode_switch_in_progress_);

  {
    std::scoped_lock lock(ecat_mtx_, hw_state_mtx_);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      Stop(out_somanet_[i], true);
    }
    ResetSpringAdjustState();
  }

  // Reset filters for hand-guided mode
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (new_modes[i] == ControlLevel::kHandGuided) {
      if (i == kWristPitchIdx) {
        wrist_pitch_dial_filter_.Reset();
        wrist_pitch_dial_state_.Reset();
      } else if (i == kWristRollIdx) {
        wrist_roll_dial_filter_.Reset();
        wrist_roll_admittance_filter_.Reset();
      } else if (i == kYaw1Idx) {
        yaw1_torque_filter_.Reset();
        yaw1_collision_torque_filter_.Reset();
      } else if (i == kYaw2Idx) {
        yaw2_torque_filter_.Reset();
      } else if (i == kWristYawIdx) {
        wrist_yaw_torque_filter_.Reset();
      } else if (i == kElevationInertialIdx) {
        elevation_inertial_torque_filter_.Reset();
      }

      std::int16_t valid_offset_state = 2;
      int result;
      {
        std::lock_guard<std::mutex> lock(ecat_mtx_);
        result = ec_SDOwrite(static_cast<std::uint16_t>(i + 1), 0x2009, 0x01,
                             false, sizeof(valid_offset_state),
                             &valid_offset_state, EC_TIMEOUTRXM);
      }
      if (result <= 0) {
        return std::unexpected(
            Error{ErrorCode::kEtherCATError,
                  "Failed to set commutation offset for joint " +
                      std::to_string(i)});
      }
    }
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    threadsafe_commands_positions_[i] =
        std::numeric_limits<float>::quiet_NaN();
    threadsafe_commands_velocities_[i] = 0.0f;
    last_velocity_write_time_ns_[i].store(0, std::memory_order_relaxed);
    threadsafe_commands_efforts_[i] =
        std::numeric_limits<float>::quiet_NaN();
    hold_in_shutdown_[i] = false;
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (new_modes[i] == ControlLevel::kSpringAdjust) {
      prevent_mode_change_.SetBlocked(ModeChangeBlockReason::kSpringAdjust);
    }
  }

  {
    std::lock_guard<std::mutex> lock(hw_state_mtx_);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      control_level_[i] = new_modes[i];
      // Keep base control_mode_ in sync for CiA402 state machine
      control_mode_[i] = ToControlMode(new_modes[i]);
    }
  }

  osal_usleep(kCyclicLoopSleepUs);

  {
    std::scoped_lock lock(ecat_mtx_, hw_state_mtx_);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (control_level_[i] != ControlLevel::kUndefined &&
          control_level_[i] != ControlLevel::kQuickStop &&
          control_level_[i] != ControlLevel::kHandGuided) {
        out_somanet_[i]->Controlword = kNormalOpBrakesOff;
      }
    }
  }

  require_new_command_mode_ = false;
  return {};
}

// ============================================================================
// Streaming commands (JointFloatArray overloads)
// ============================================================================

std::expected<void, Error> ArmInterface::SetPositionCommand(
    const JointFloatArray& positions,
    std::function<bool()> halt_condition) {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  if (!IsSpringStreamingCommandSlotUnused(positions[kSpringAdjustIdx])) {
    return std::unexpected(
        Error{ErrorCode::kInvalidArgument,
              "spring_adjust_joint slot must be NaN; use SetSpringSetpoint "
              "for load"});
  }

  bool need_switch = false;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i == kSpringAdjustIdx) {
      if (control_level_[i] != ControlLevel::kSpringAdjust) {
        need_switch = true;
        break;
      }
    } else if (control_level_[i] != ControlLevel::kPosition) {
      need_switch = true;
      break;
    }
  }
  if (need_switch) {
    auto result = SwitchControlMode(MakeStreamingModes(ControlLevel::kPosition));
    if (!result) return result;
  }

  {
    std::lock_guard<std::mutex> lock(halt_mutex_);
    halt_condition_ = std::move(halt_condition);
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i == kSpringAdjustIdx) continue;
    float pos_red = position_reductions_[i].load();
    std::uint32_t enc_res = encoder_resolutions_[i].load();
    threadsafe_commands_positions_[i] = static_cast<float>(
        OutputShaftRadToInputTicks(positions[i], pos_red, enc_res,
                                   startup_angle_wrap_value_[i]));
  }
  return {};
}

std::expected<void, Error> ArmInterface::SetVelocityCommand(
    const JointFloatArray& velocities,
    std::function<bool()> halt_condition) {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  if (!IsSpringStreamingCommandSlotUnused(velocities[kSpringAdjustIdx])) {
    return std::unexpected(
        Error{ErrorCode::kInvalidArgument,
              "spring_adjust_joint slot must be NaN; use SetSpringSetpoint "
              "for load"});
  }

  bool need_switch = false;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i == kSpringAdjustIdx) {
      if (control_level_[i] != ControlLevel::kSpringAdjust) {
        need_switch = true;
        break;
      }
    } else if (control_level_[i] != ControlLevel::kVelocity) {
      need_switch = true;
      break;
    }
  }
  if (need_switch) {
    auto result = SwitchControlMode(MakeStreamingModes(ControlLevel::kVelocity));
    if (!result) return result;
  }

  {
    std::lock_guard<std::mutex> lock(halt_mutex_);
    halt_condition_ = std::move(halt_condition);
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i == kSpringAdjustIdx) continue;
    threadsafe_commands_velocities_[i] = static_cast<float>(
        OutputShaftRadPerSToVelocityValue(velocities[i],
                                          si_velocity_units_[i].load()));
  }
  const int64_t now_ns =
      std::chrono::steady_clock::now().time_since_epoch().count();
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i == kSpringAdjustIdx) continue;
    last_velocity_write_time_ns_[i].store(now_ns, std::memory_order_release);
  }
  return {};
}

std::expected<void, Error> ArmInterface::SetTorqueCommand(
    const JointFloatArray& torques,
    std::function<bool()> halt_condition) {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  if (!IsSpringStreamingCommandSlotUnused(torques[kSpringAdjustIdx])) {
    return std::unexpected(
        Error{ErrorCode::kInvalidArgument,
              "spring_adjust_joint slot must be NaN; use SetSpringSetpoint "
              "for load"});
  }

  bool need_switch = false;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i == kSpringAdjustIdx) {
      if (control_level_[i] != ControlLevel::kSpringAdjust) {
        need_switch = true;
        break;
      }
    } else if (control_level_[i] != ControlLevel::kTorque) {
      need_switch = true;
      break;
    }
  }
  if (need_switch) {
    auto result = SwitchControlMode(MakeStreamingModes(ControlLevel::kTorque));
    if (!result) return result;
  }

  {
    std::lock_guard<std::mutex> lock(halt_mutex_);
    halt_condition_ = std::move(halt_condition);
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i == kSpringAdjustIdx) continue;
    threadsafe_commands_efforts_[i] = std::clamp(torques[i], -1000.0f, 1000.0f);
  }
  return {};
}

// ============================================================================
// Generic commands
// ============================================================================

std::expected<void, Error> ArmInterface::SendCommand(
    const JointFloatArray& joint_commands) {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    std::uint32_t enc_res = encoder_resolutions_[i].load();

    switch (control_level_[i]) {
      case ControlLevel::kPosition:
        threadsafe_commands_positions_[i] = static_cast<float>(
            OutputShaftRadToInputTicks(joint_commands[i],
                                       position_reductions_[i].load(), enc_res,
                                       startup_angle_wrap_value_[i]));
        break;
      case ControlLevel::kVelocity:
        threadsafe_commands_velocities_[i] = static_cast<float>(
            OutputShaftRadPerSToVelocityValue(
                joint_commands[i], si_velocity_units_[i].load()));
        last_velocity_write_time_ns_[i].store(
            std::chrono::steady_clock::now().time_since_epoch().count(),
            std::memory_order_release);
        break;
      case ControlLevel::kTorque:
        threadsafe_commands_efforts_[i] =
            std::clamp(joint_commands[i], -1000.0f, 1000.0f);
        break;
      default:
        break;
    }
  }
  return {};
}

std::expected<void, Error> ArmInterface::SendCommand(
    const std::vector<JointName>& joints,
    const std::vector<float>& joint_commands) {
  if (joints.size() != joint_commands.size()) {
    return std::unexpected(
        Error{ErrorCode::kInvalidArgument,
              "joints and commands must have same size"});
  }
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  for (std::size_t j = 0; j < joints.size(); ++j) {
    auto i = JointIndex(joints[j]);
    if (i >= kNumJoints) {
      return std::unexpected(
          Error{ErrorCode::kInvalidArgument, "Invalid joint index"});
    }
    std::uint32_t enc_res = encoder_resolutions_[i].load();

    switch (control_level_[i]) {
      case ControlLevel::kPosition:
        threadsafe_commands_positions_[i] = static_cast<float>(
            OutputShaftRadToInputTicks(joint_commands[j],
                                       position_reductions_[i].load(), enc_res,
                                       startup_angle_wrap_value_[i]));
        break;
      case ControlLevel::kVelocity:
        threadsafe_commands_velocities_[i] = static_cast<float>(
            OutputShaftRadPerSToVelocityValue(
                joint_commands[j], si_velocity_units_[i].load()));
        last_velocity_write_time_ns_[i].store(
            std::chrono::steady_clock::now().time_since_epoch().count(),
            std::memory_order_release);
        break;
      case ControlLevel::kTorque:
        threadsafe_commands_efforts_[i] =
            std::clamp(joint_commands[j], -1000.0f, 1000.0f);
        break;
      default:
        break;
    }
  }
  return {};
}

// ============================================================================
// Spring
// ============================================================================

std::expected<void, Error> ArmInterface::SetSpringSetpoint(
    float load_in_newtons) {
  if ((load_in_newtons < 0) || (load_in_newtons > 882)) {
    return std::unexpected(Error{ErrorCode::kInvalidSpringSetpoint,
                                 "Invalid spring setpoint"});
  }
  const float target_ticks = ConvertNewtonsToSpringLipsTicks(load_in_newtons);
  StoreSpringAdjustSetpoint(spring_setpoint_target_ticks_,
                            has_spring_setpoint_, target_ticks);

  const bool spring_adjust_active =
      control_level_[kSpringAdjustIdx] == ControlLevel::kSpringAdjust;
  const bool needs_fresh_session =
      !spring_adjust_active ||
      !prevent_mode_change_.IsBlocked(ModeChangeBlockReason::kSpringAdjust);
  if (!needs_fresh_session || !control_loop_running_) {
    return {};
  }

  JointControlLevelArray new_modes = control_level_;
  new_modes[kSpringAdjustIdx] = ControlLevel::kSpringAdjust;
  return SwitchControlMode(new_modes);
}

// ============================================================================
// State queries (JointFloatArray wrappers)
// ============================================================================

std::expected<JointFloatArray, Error> ArmInterface::GetPositionsArray() const {
  auto result = GetPositions();
  if (!result) return std::unexpected(result.error());
  JointFloatArray arr{};
  std::copy_n(result->begin(), kNumJoints, arr.begin());
  return arr;
}

std::expected<JointFloatArray, Error> ArmInterface::GetVelocitiesArray() const {
  auto result = GetVelocities();
  if (!result) return std::unexpected(result.error());
  JointFloatArray arr{};
  std::copy_n(result->begin(), kNumJoints, arr.begin());
  return arr;
}

std::expected<JointFloatArray, Error> ArmInterface::GetTorquesArray() const {
  auto result = GetTorques();
  if (!result) return std::unexpected(result.error());
  JointFloatArray arr{};
  std::copy_n(result->begin(), kNumJoints, arr.begin());
  return arr;
}

std::expected<JointFloatArray, Error> ArmInterface::GetAccelerationsArray() const {
  auto result = GetAccelerations();
  if (!result) return std::unexpected(result.error());
  JointFloatArray arr{};
  std::copy_n(result->begin(), kNumJoints, arr.begin());
  return arr;
}

// ============================================================================
// GPIO
// ============================================================================

std::expected<std::vector<bool>, Error> ArmInterface::GetButtonState() const {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  return std::vector<bool>{
      hw_function_enable_.load() > 0.5f,
      hw_comp_button_.load() > 0.5f,
      hw_decomp_button_.load() > 0.5f,
  };
}

std::expected<std::vector<int>, Error> ArmInterface::GetDialState() const {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  return std::vector<int>{0, 0};
}

// ============================================================================
// Joint limits
// ============================================================================

std::expected<JointArray<JointLimitsInfo>, Error>
ArmInterface::GetJointLimitsArray() const {
  JointArray<JointLimitsInfo> limits{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    limits[i].has_limits = has_position_limits_[i];
    limits[i].min_position = min_position_limits_[i];
    limits[i].max_position = max_position_limits_[i];
  }
  return limits;
}

// ============================================================================
// Internal helpers
// ============================================================================

void ArmInterface::ResetSpringAdjustState() {
  spring_adjust_state_.time_prev = std::chrono::steady_clock::now();
  spring_adjust_state_.error_prev = std::nullopt;
}

bool ArmInterface::IsEStopEngaged() {
  // Call base implementation for PDO exchange + WKC check
  if (SynapticonBase::IsEStopEngaged()) {
    return true;
  }

  // Arm-specific: SDO-based e-stop read
  std::uint16_t slave_number = 1;
  bool value_holder;
  int object_size = sizeof(value_holder);
  int result = ec_SDOread(slave_number, 0x6621, 0x01, FALSE, &object_size,
                          &value_holder, EC_TIMEOUTRXM);
  if (result <= 0) {
    spdlog::error("Failed to read e-stop status");
    return true;
  }
  return value_holder;
}

void ArmInterface::OnPreStateMachine() {
  skip_cycle_ = false;

  auto wr_roll_gpio = ReadSDOValue(
      static_cast<std::uint16_t>(kWristRollIdx + 1), 0x60FD, 0x00);
  if (!wr_roll_gpio) {
    spdlog::error("ReadSDOValue() failed for GPIO");
    skip_cycle_ = true;
    return;
  }

  lips_spring_position_ = in_somanet_[kSpringAdjustIdx]->PositionValue;
  // Synapticon applies angle-wrapping to LIPS sometimes. Undo that.
  constexpr int32_t LIPS_ENCODER_RESOLUTION = 4096;
  if (lips_spring_position_ < 0)
  {
    lips_spring_position_ += LIPS_ENCODER_RESOLUTION;
  }
  else if (lips_spring_position_ >= LIPS_ENCODER_RESOLUTION)
  {
    lips_spring_position_ -= LIPS_ENCODER_RESOLUTION;
  }

  hw_function_enable_ =
      static_cast<float>((*wr_roll_gpio & (1 << 16)) >> 16);
  hw_decomp_button_ =
      static_cast<float>((*wr_roll_gpio & (1 << 17)) >> 17);
  hw_comp_button_ =
      static_cast<float>((*wr_roll_gpio & (1 << 18)) >> 18);
  admittance_button_pressed_ = (*wr_roll_gpio & (1 << 19)) >> 19;

  deadman_pressed_ = hw_function_enable_.load() > 0.5f;

  if (first_iteration_deadman_ && deadman_pressed_) {
    first_iteration_deadman_ = false;
    prevent_mode_change_.SetBlocked(ModeChangeBlockReason::kStuckFunctionEnable);
    prev_deadman_pressed_ = true;
    spdlog::error(
        "Function enable pressed at startup - blocking hand_guided");
    skip_cycle_ = true;
    return;
  }
  first_iteration_deadman_ = false;

  if (require_new_command_mode_) {
    if (!deadman_pressed_) {
      prevent_mode_change_.ClearBlocked(
          ModeChangeBlockReason::kStuckFunctionEnable);
    }
    skip_cycle_ = true;
    return;
  }

  float payload_mass_kg = SpringPotTicksToPayloadKg(lips_spring_position_);
  if (payload_mass_kg > 0) {
    dynamic_sim_state_->input.payload_mass = payload_mass_kg;
  } else {
    dynamic_sim_state_->input.payload_mass = 0.0f;
  }

  if (!deadman_pressed_) {
    prevent_mode_change_.ClearBlocked(
        ModeChangeBlockReason::kStuckFunctionEnable);
  }

  // Rising edge of the deadman in hand-guided: clear the hold-in-shutdown
  // latches set by OnNormalOperation while the handle was released, so the
  // CiA402 state machine can walk the drives back up to Operation Enabled.
  // OnNormalOperation can't do this itself because it only runs once the
  // drive is already in Operation Enabled.
  if (control_level_[0] == ControlLevel::kHandGuided &&
      deadman_pressed_ && !prev_deadman_pressed_) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      hold_in_shutdown_[i] = false;
    }
  }

  UpdateDeadmanModeChangeState(control_level_[0], deadman_pressed_,
                               prev_deadman_pressed_, prevent_mode_change_);
}

bool ArmInterface::OnNormalOperation(std::size_t joint_idx, bool mode_switch) {
  if (skip_cycle_) return true;

  // Apply gravity comp for all joints in hand-guided and spring-adjust modes
  if (control_level_[joint_idx] == ControlLevel::kHandGuided ||
      control_level_[joint_idx] == ControlLevel::kSpringAdjust) {
    ApplyGravComp(joint_idx, in_somanet_[joint_idx], out_somanet_[joint_idx]);
  }

  // --- Hand-guided ---
  if (control_level_[joint_idx] == ControlLevel::kHandGuided) {
    if (!deadman_pressed_) {
      if (joint_idx == kWristPitchIdx) {
        wrist_pitch_dial_filter_.Reset();
        wrist_pitch_dial_state_.Reset();
      } else if (joint_idx == kWristRollIdx) {
        wrist_roll_dial_filter_.Reset();
        wrist_roll_admittance_filter_.Reset();
      }
      // Latch shutdown so HandleSwitchOn / HandleEnableOperation keep the
      // drive out of Operation Enabled while the deadman is released.
      // Cleared on the rising edge of deadman in OnPreStateMachine.
      hold_in_shutdown_[joint_idx] = true;
      if (!mode_switch) {
        Stop(out_somanet_[joint_idx], true);
      }
      return true;
    }

    if ((joint_idx != kSpringAdjustIdx) &&
        (joint_idx != kElevationInertialIdx)) {
      if (!admittance_button_pressed_ && joint_admittances_[joint_idx]) {
        wrist_roll_admittance_filter_.Reset();
      }

      // Admittance mode
      if (admittance_button_pressed_ && joint_admittances_[joint_idx]) {
        float joint_vel = 0.0f;
        if (!std::isnan(
                dynamic_sim_state_->output.tau_required.at(joint_idx)) &&
            !std::isnan(dynamic_sim_state_->output.estimated_accelerations
                            [joint_idx])) {
          float T_ext = in_somanet_[joint_idx]->TorqueValue -
                        dynamic_sim_state_->output.tau_required.at(joint_idx);
          float joint_accel =
              dynamic_sim_state_->output.estimated_accelerations[joint_idx];
          joint_vel = -kAdmittanceVelocityMultiplier *
                      mechanical_reductions_[joint_idx].load() *
                      joint_admittances_[joint_idx]->CalculateVelocity(
                          joint_accel, T_ext);
        } else {
          wrist_roll_admittance_filter_.Reset();
        }
        const auto has_pos_lim = HasPositionLimitsToArray(has_position_limits_);
        SetVelocityWithLimits(joint_idx, in_somanet_[joint_idx],
                              has_pos_lim, min_position_limits_,
                              max_position_limits_, max_brake_torques_,
                              position_reductions_[joint_idx].load(),
                              encoder_resolutions_[joint_idx].load(), joint_vel,
                              si_velocity_units_[joint_idx].load(),
                              startup_angle_wrap_value_[joint_idx],
                              wrist_roll_admittance_filter_,
                              out_somanet_[joint_idx]);
        out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
      }
      // Wrist pitch (dial controlled)
      else if (joint_idx == kWristPitchIdx) {
        std::optional<std::int32_t> wrist_pitch_dial_value = ReadSDOValue(
            static_cast<std::uint16_t>(kWristRollIdx + 1), 0x2402, 0x00);
        if (!wrist_pitch_dial_value) {
          spdlog::error("ReadSDOValue() failed for pitch dial");
          Stop(out_somanet_[joint_idx], true);
          hold_in_shutdown_[joint_idx] = true;
          wrist_pitch_dial_filter_.Reset();
          wrist_pitch_dial_state_.Reset();
          return true;
        }

        *wrist_pitch_dial_value = std::clamp(
            *wrist_pitch_dial_value,
            elevate_config_.button_config.pitch_dial.min,
            elevate_config_.button_config.pitch_dial.max);
        float normalized_dial =
            -static_cast<float>(
                *wrist_pitch_dial_value -
                elevate_config_.button_config.pitch_dial.center) /
            (0.5f * (elevate_config_.button_config.pitch_dial.max -
                     elevate_config_.button_config.pitch_dial.min));

        const bool was_in_shutdown = hold_in_shutdown_[joint_idx].load();
        const bool dial_is_active = UpdateWristPitchDialActivationState(
            normalized_dial, wrist_pitch_dial_state_);
        const float velocity = dial_is_active
                                   ? NormalizeWristPitchDialToVelocity(
                                         normalized_dial)
                                   : 0.0f;
        hold_in_shutdown_[joint_idx] = !dial_is_active;

        if (hold_in_shutdown_[joint_idx].load()) {
          Stop(out_somanet_[joint_idx], true);
          wrist_pitch_dial_filter_.Reset();
          wrist_pitch_dial_state_.Reset();
        } else {
          if (was_in_shutdown && !mode_switch) {
            out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
          }
          const auto has_pos_lim2 = HasPositionLimitsToArray(has_position_limits_);
          SetVelocityWithLimits(
              joint_idx, in_somanet_[joint_idx], has_pos_lim2,
              min_position_limits_, max_position_limits_, max_brake_torques_,
              position_reductions_[joint_idx].load(),
              encoder_resolutions_[joint_idx].load(), velocity,
              si_velocity_units_[joint_idx].load(),
              startup_angle_wrap_value_[joint_idx],
              wrist_pitch_dial_filter_, out_somanet_[joint_idx]);
        }
      }
      // Wrist roll (dial controlled)
      else if (joint_idx == kWristRollIdx) {
        std::optional<std::int32_t> wrist_roll_dial_value = ReadSDOValue(
            static_cast<std::uint16_t>(kWristYawIdx + 1), 0x2402, 0x00);
        if (!wrist_roll_dial_value) {
          spdlog::error("ReadSDOValue() failed for roll dial");
          Stop(out_somanet_[joint_idx], true);
          wrist_roll_dial_filter_.Reset();
          return true;
        }

        *wrist_roll_dial_value = std::clamp(
            *wrist_roll_dial_value,
            elevate_config_.button_config.roll_dial.min,
            elevate_config_.button_config.roll_dial.max);
        const float normalized_dial =
            static_cast<float>(*wrist_roll_dial_value -
                               elevate_config_.button_config.roll_dial.center) /
            (0.5f * (elevate_config_.button_config.roll_dial.max -
                     elevate_config_.button_config.roll_dial.min));
        const float velocity =
            NormalizeWristRollDialToVelocity(normalized_dial);

        const auto has_pos_lim3 = HasPositionLimitsToArray(has_position_limits_);
        SetVelocityWithLimits(joint_idx, in_somanet_[joint_idx],
                              has_pos_lim3, min_position_limits_,
                              max_position_limits_, max_brake_torques_,
                              position_reductions_[joint_idx].load(),
                              encoder_resolutions_[joint_idx].load(), velocity,
                              si_velocity_units_[joint_idx].load(),
                              startup_angle_wrap_value_[joint_idx],
                              wrist_roll_dial_filter_,
                              out_somanet_[joint_idx]);
        if (!mode_switch) {
          out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
        }
      }
      // Remaining joints: torque mode
      else {
        float user_torque =
            CalculateUserTorque(in_somanet_[joint_idx], joint_idx);
        out_somanet_[joint_idx]->TorqueOffset += static_cast<std::int16_t>(
            kTorqueNmToPerMilleMultiplier *
            TorqueNmToTorquePerMille(user_torque,
                                     mechanical_reductions_[joint_idx],
                                     rated_torques_[joint_idx]));

        float torque = 0.0f;
        float current_position = InputTicksToOutputShaftRad(
            in_somanet_[joint_idx]->PositionValue,
            position_reductions_[joint_idx].load(),
            encoder_resolutions_[joint_idx].load(),
            startup_angle_wrap_value_[joint_idx]);
        const auto has_pos_lim4 = HasPositionLimitsToArray(has_position_limits_);
        auto result = JointLimitCmdClamp(
            joint_idx, ControlMode::kTorque, current_position,
            has_pos_lim4, min_position_limits_, max_position_limits_,
            max_brake_torques_, torque);
        if (result.has_value()) {
          torque = result->second;
          if (joint_idx == kWristYawIdx) {
            float soft_limit_start = kWristYawHandGuidedSoftLimit -
                                     kWristYawHandGuidedBrakeRamp;
            float abs_pos = std::abs(current_position);
            if (abs_pos > soft_limit_start) {
              float fraction = std::clamp(
                  (abs_pos - soft_limit_start) / kWristYawHandGuidedBrakeRamp,
                  0.0f, 1.0f);
              float brake = fraction * kWristYawHandGuidedMaxBrakeTorque;
              torque += (current_position > 0.0f) ? -brake : brake;
            }
          }
          if (joint_idx == kYaw1Idx) {
            torque *= elevate_config_.torque_sign.yaw1;
          } else if (joint_idx == kYaw2Idx) {
            torque *= elevate_config_.torque_sign.yaw2;
          } else if (joint_idx == kWristYawIdx) {
            torque *= elevate_config_.torque_sign.wrist_yaw;
          }
        }

        ApplyFrictionCompensation(joint_idx);

        if (joint_idx == kYaw1Idx) {
          out_somanet_[joint_idx]->TorqueOffset = static_cast<std::int16_t>(
              yaw1_torque_filter_.Filter(out_somanet_[joint_idx]->TorqueOffset));
        } else if (joint_idx == kYaw2Idx) {
          out_somanet_[joint_idx]->TorqueOffset = static_cast<std::int16_t>(
              yaw2_torque_filter_.Filter(out_somanet_[joint_idx]->TorqueOffset));
        } else if (joint_idx == kWristYawIdx) {
          out_somanet_[joint_idx]->TorqueOffset =
              static_cast<std::int16_t>(wrist_yaw_torque_filter_.Filter(
                  out_somanet_[joint_idx]->TorqueOffset));
        }
        out_somanet_[joint_idx]->TargetTorque =
            static_cast<std::int16_t>(torque);
        out_somanet_[joint_idx]->OpMode = kProfileTorqueMode;
        if (!mode_switch) {
          out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
        }
      }
    }
    // Elevation link joints
    else {
      if (joint_idx == kElevationInertialIdx) {
        float user_torque =
            CalculateUserTorque(in_somanet_[joint_idx], joint_idx);
        out_somanet_[joint_idx]->TorqueOffset += static_cast<std::int16_t>(
            kTorqueNmToPerMilleMultiplier *
            TorqueNmToTorquePerMille(user_torque,
                                     mechanical_reductions_[joint_idx],
                                     rated_torques_[joint_idx]));
        out_somanet_[joint_idx]->TorqueOffset =
            static_cast<std::int16_t>(elevation_inertial_torque_filter_.Filter(
                out_somanet_[joint_idx]->TorqueOffset));
      }

      out_somanet_[joint_idx]->TargetTorque = 0;
      out_somanet_[joint_idx]->OpMode = kProfileTorqueMode;
      ApplyFrictionCompensation(joint_idx);
      if (!mode_switch) {
        out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
      }
    }
    return true;
  }

  // --- Spring adjust ---
  if (control_level_[joint_idx] == ControlLevel::kSpringAdjust) {
    if (joint_idx == kSpringAdjustIdx) {
      if (prevent_mode_change_.IsBlocked(ModeChangeBlockReason::kSpringAdjust)) {
        bool spring_adjust_complete = false;
        const auto target = LoadSpringAdjustSetpoint(
            spring_setpoint_target_ticks_, has_spring_setpoint_);
        if (target.has_value()) {
          float actuator_torque = SpringAdjustByLIPS(
              *target, lips_spring_position_, spring_adjust_complete,
              spring_adjust_state_);
          if (spring_adjust_complete) {
            prevent_mode_change_.ClearBlocked(
                ModeChangeBlockReason::kSpringAdjust);
          }
          out_somanet_[joint_idx]->TargetTorque =
              static_cast<std::int16_t>(actuator_torque);
          out_somanet_[joint_idx]->OpMode = kProfileTorqueMode;
          out_somanet_[joint_idx]->TorqueOffset = 0;
          if (!mode_switch) {
            out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
          }
        } else {
          spdlog::warn("A spring adjust setpoint wasn't defined for the spring adjust joint. Stopping.");
          control_level_[joint_idx] = ControlLevel::kQuickStop;
          control_mode_[joint_idx] = ControlMode::kQuickStop;
        }
      } else {
        ControlMode cm = ControlMode::kQuickStop;
        CompleteSpringAdjustSession(has_spring_setpoint_, cm);
        control_level_[joint_idx] = ControlLevel::kQuickStop;
        control_mode_[joint_idx] = cm;
      }
    } else {
      Stop(out_somanet_[joint_idx], true);
    }
    return true;
  }

  // For base modes in hand-guided pitch, check hold state
  if (joint_idx == kWristPitchIdx &&
      control_level_[joint_idx] == ControlLevel::kHandGuided &&
      hold_in_shutdown_[joint_idx].load()) {
    RefreshHandGuidedPitchBrakeHold();
  }

  // Not handled by arm — let base handle it
  return false;
}

float ArmInterface::CalculateUserTorque(const InSomanet50t* in_somanet,
                                        std::size_t joint_idx) {
  float measured_torque_nm = TorquePerMilleToTorqueNm(
      in_somanet->TorqueValue, mechanical_reductions_[joint_idx],
      rated_torques_[joint_idx]);
  float expected_torque = dynamic_sim_state_->output.tau_required.at(joint_idx);
  return expected_torque - measured_torque_nm;
}

void ArmInterface::ApplyGravComp(std::size_t joint_idx,
                                 const InSomanet50t* in_somanet,
                                 OutSomanet50t* out_somanet) {
  if ((joint_idx == kYaw1Idx) || (joint_idx == kYaw2Idx) ||
      (joint_idx == kWristYawIdx) || (joint_idx == kElevationInertialIdx)) {
    out_somanet->TorqueOffset = static_cast<std::int16_t>(
        kTorqueNmToPerMilleMultiplier *
        TorqueNmToTorquePerMille(
            dynamic_sim_state_->output.tau_required.at(joint_idx),
            mechanical_reductions_[joint_idx], rated_torques_[joint_idx]));
  } else if (joint_idx == kWristPitchIdx) {
    ApplyWristPitchHoldTorque(in_somanet, out_somanet);
  } else if (joint_idx == kWristRollIdx) {
    out_somanet->TorqueOffset = 0;
  }
}

void ArmInterface::ApplyWristPitchHoldTorque(const InSomanet50t* in_somanet,
                                             OutSomanet50t* out_somanet) {
  float current_position = InputTicksToOutputShaftRad(
      in_somanet->PositionValue, position_reductions_[kWristPitchIdx].load(),
      encoder_resolutions_[kWristPitchIdx].load(),
      startup_angle_wrap_value_[kWristPitchIdx]);
  float cosine_term = kWristPitchHoldTorque * std::cos(current_position);
  float drake_ff = kTorqueNmToPerMilleMultiplier *
                   TorqueNmToTorquePerMille(
                       dynamic_sim_state_->output.tau_required.at(kWristPitchIdx),
                       mechanical_reductions_[kWristPitchIdx],
                       rated_torques_[kWristPitchIdx]);
  out_somanet->TorqueOffset = static_cast<std::int16_t>(cosine_term + drake_ff);
}

void ArmInterface::RefreshHandGuidedPitchBrakeHold() {
  std::optional<std::int32_t> wrist_pitch_dial_value =
      ReadSDOValue(static_cast<std::uint16_t>(kWristRollIdx + 1), 0x2402, 0x00);
  if (!wrist_pitch_dial_value) {
    spdlog::error("ReadSDOValue() failed for pitch dial");
    hold_in_shutdown_[kWristPitchIdx] = true;
    wrist_pitch_dial_filter_.Reset();
    wrist_pitch_dial_state_.Reset();
    return;
  }

  *wrist_pitch_dial_value = std::clamp(
      *wrist_pitch_dial_value, elevate_config_.button_config.pitch_dial.min,
      elevate_config_.button_config.pitch_dial.max);
  float normalized_dial =
      -static_cast<float>(
          *wrist_pitch_dial_value - elevate_config_.button_config.pitch_dial.center) /
      (0.5f * (elevate_config_.button_config.pitch_dial.max -
               elevate_config_.button_config.pitch_dial.min));
  hold_in_shutdown_[kWristPitchIdx] =
      !UpdateWristPitchDialActivationState(normalized_dial,
                                           wrist_pitch_dial_state_);
}

void ArmInterface::ApplyFrictionCompensation(std::size_t joint_idx) {
  float joint_velocity_rad_s = VelocityValueToOutputShaftRadPerS(
      in_somanet_[joint_idx]->VelocityValue,
      si_velocity_units_[joint_idx].load(),
      mechanical_reductions_[joint_idx].load());

  std::int32_t torque_sign = 0;
  if (joint_idx == kYaw1Idx) {
    torque_sign = elevate_config_.torque_sign.yaw1;
  } else if (joint_idx == kYaw2Idx) {
    torque_sign = elevate_config_.torque_sign.yaw2;
  } else if (joint_idx == kWristYawIdx) {
    torque_sign = elevate_config_.torque_sign.wrist_yaw;
  }

  float yaw2_rad = InputTicksToOutputShaftRad(
      in_somanet_[kYaw2Idx]->PositionValue,
      position_reductions_[kYaw2Idx].load(),
      encoder_resolutions_[kYaw2Idx].load(),
      startup_angle_wrap_value_[kYaw2Idx]);

  if (joint_idx == kYaw1Idx) {
    if (std::abs(yaw2_rad) < kYawAlignmentFrictionThreshold) {
      if (joint_velocity_rad_s > 0) {
        out_somanet_[joint_idx]->TorqueOffset +=
            -torque_sign * kTorqueFrictionOffset[joint_idx];
      } else if (joint_velocity_rad_s < 0) {
        out_somanet_[joint_idx]->TorqueOffset +=
            torque_sign * kTorqueFrictionOffset[joint_idx];
      }
    } else {
      if (joint_velocity_rad_s > 0) {
        out_somanet_[joint_idx]->TorqueOffset +=
            torque_sign * kTorqueFrictionOffset[joint_idx];
      } else if (joint_velocity_rad_s < 0) {
        out_somanet_[joint_idx]->TorqueOffset +=
            -torque_sign * kTorqueFrictionOffset[joint_idx];
      }
    }
    return;
  }

  if (joint_idx == kYaw2Idx) {
    return;
  }

  if (joint_velocity_rad_s > kFrictionDeadband) {
    out_somanet_[joint_idx]->TorqueOffset +=
        torque_sign * kTorqueFrictionOffset[joint_idx];
  } else if (joint_velocity_rad_s < -kFrictionDeadband) {
    out_somanet_[joint_idx]->TorqueOffset +=
        -torque_sign * kTorqueFrictionOffset[joint_idx];
  }
}

}  // namespace elevated_control
