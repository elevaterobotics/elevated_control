// Copyright (c) 2025 Elevate Robotics Inc

#include "elevated_control/arm_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "elevated_control/joint_limits.hpp"
#include "elevated_control/state_machine.hpp"
#include "elevated_control/unit_conversions.hpp"

using namespace std::chrono_literals;

namespace elevated_control {

namespace {

constexpr int kEcTimeoutMon = 500;
constexpr char kExpectedSlaveName[] = "SOMANET";

bool InterfaceExists(const char* ifname) {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) return false;

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  bool exists = (ioctl(sock, SIOCGIFINDEX, &ifr) == 0);
  close(sock);
  return exists;
}

}  // namespace

// SDO read (defined in state_machine.hpp, implemented here)
std::optional<std::int32_t> ReadSDOValue(std::uint16_t slave,
                                         std::uint16_t index,
                                         std::uint8_t subindex) {
  std::int32_t value_holder;
  int object_size = sizeof(value_holder);
  int result = ec_SDOread(slave, index, subindex, FALSE, &object_size,
                          &value_holder, EC_TIMEOUTRXM);
  if (result <= 0) return std::nullopt;
  return value_holder;
}

// ============================================================================
// Construction / Destruction
// ============================================================================

ArmInterface::ArmInterface(const Config& config)
    : config_(config),
      dynamic_sim_state_(std::make_shared<DynamicSimState>()),
      dynamic_simulator_(dynamic_sim_state_) {}

ArmInterface::~ArmInterface() {
  if (control_loop_running_) {
    auto result = StopControlLoop();
    if (!result) {
      spdlog::error("StopControlLoop failed in destructor: {}",
                    result.error().message);
    }
  }

  if (initialized_) {
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    std::this_thread::sleep_for(1000ms);
    ec_close();
  }
}

// ============================================================================
// Lifecycle
// ============================================================================

std::expected<void, Error> ArmInterface::Initialize() {
  if (initialized_) {
    return std::unexpected(
        Error{ErrorCode::kAlreadyInitialized, "Already initialized"});
  }

  // Initialize command buffers
  control_level_.resize(kNumJoints, ControlLevel::kUndefined);
  state_positions_.resize(kNumJoints, std::numeric_limits<float>::quiet_NaN());
  state_velocities_.resize(kNumJoints, std::numeric_limits<float>::quiet_NaN());
  state_torques_.resize(kNumJoints, std::numeric_limits<float>::quiet_NaN());
  state_accelerations_.resize(kNumJoints, std::numeric_limits<float>::quiet_NaN());

  mechanical_reductions_.resize(kNumJoints);
  encoder_resolutions_.resize(kNumJoints);
  rated_torques_.resize(kNumJoints);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    mechanical_reductions_[i] = 1.0f;
    encoder_resolutions_[i] = 1;
    rated_torques_[i] = 1.0f;
  }

  threadsafe_commands_positions_.resize(kNumJoints);
  threadsafe_commands_velocities_.resize(kNumJoints);
  threadsafe_commands_efforts_.resize(kNumJoints);
  threadsafe_commands_spring_adjust_.resize(kNumJoints);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    threadsafe_commands_positions_[i] =
        std::numeric_limits<float>::quiet_NaN();
    threadsafe_commands_velocities_[i] = 0.0f;
    threadsafe_commands_efforts_[i] =
        std::numeric_limits<float>::quiet_NaN();
    threadsafe_commands_spring_adjust_[i] =
        std::numeric_limits<float>::quiet_NaN();
  }

  // Initialize dynamic sim state (use kNumJoints entries for the stub)
  InitializeDynamicSimState(dynamic_sim_state_, kNumJoints);

  // Parse joint limits
  if (!config_.joint_limits_yaml.empty()) {
    auto joint_limits_config =
        ParseJointLimits(config_.joint_limits_yaml, kJointNames);
    if (!ValidateJointLimits(joint_limits_config)) {
      return std::unexpected(
          Error{ErrorCode::kInvalidArgument, "Invalid joint limits config"});
    }
    has_position_limits_ = std::move(joint_limits_config.has_position_limits);
    min_position_limits_ = std::move(joint_limits_config.min_position_limits);
    max_position_limits_ = std::move(joint_limits_config.max_position_limits);
  } else {
    has_position_limits_.resize(kNumJoints, false);
    min_position_limits_.resize(kNumJoints,
                                -std::numeric_limits<float>::max());
    max_position_limits_.resize(kNumJoints,
                                std::numeric_limits<float>::max());
  }

  // Parse elevate config
  if (!config_.elevate_config_yaml.empty()) {
    elevate_config_ = ParseElevateConfig(config_.elevate_config_yaml);
    if (!ValidateButtonConfig(elevate_config_.button_config) ||
        !ValidateSpringSetpoints(elevate_config_.spring_setpoints) ||
        !ValidateTorqueSignsConfig(elevate_config_.torque_sign) ||
        !ValidatePositionSignsConfig(elevate_config_.position_sign) ||
        !ValidateVelocitySignsConfig(elevate_config_.velocity_sign)) {
      return std::unexpected(
          Error{ErrorCode::kInvalidArgument, "Invalid elevate config"});
    }
  }

  // Check network interface
  if (!InterfaceExists(config_.network_interface.c_str())) {
    return std::unexpected(Error{
        ErrorCode::kEtherCATError,
        "Network interface '" + config_.network_interface + "' does not exist"});
  }

  // EtherCAT initialization
  int ec_init_status = ec_init(config_.network_interface.c_str());
  if (ec_init_status <= 0) {
    ec_close();
    return std::unexpected(Error{ErrorCode::kEtherCATError,
                                 "EtherCAT init failed on " +
                                     config_.network_interface});
  }

  if (ec_config_init(false) <= 0) {
    ec_close();
    return std::unexpected(
        Error{ErrorCode::kEtherCATError, "No EtherCAT slaves found"});
  }

  ec_config_map(&io_map_);
  ec_configdc();
  WaitForGoodProcessData();
  HandleStartupFaults();

  // Request operational state for all slaves
  expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  for (int slave_id = 0; slave_id < ec_slavecount; ++slave_id) {
    ec_slave[slave_id].state = EC_STATE_OPERATIONAL;
  }
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  ec_writestate(0);

  // Wait for all slaves to reach OP state
  uint16 actual_state;
  std::size_t chk = 200;
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_readstate();
    actual_state = ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  } while (chk-- && (actual_state != EC_STATE_OPERATIONAL));

  if (chk == 0) {
    ec_close();
    return std::unexpected(Error{ErrorCode::kEtherCATError,
                                 "Slaves failed to reach OPERATIONAL state"});
  }

  // Connect PDO struct pointers
  in_somanet_.clear();
  out_somanet_.clear();
  for (std::size_t joint_idx = 1; joint_idx <= kNumJoints; ++joint_idx) {
    in_somanet_.push_back(
        reinterpret_cast<InSomanet50t*>(ec_slave[joint_idx].inputs));
    out_somanet_.push_back(
        reinterpret_cast<OutSomanet50t*>(ec_slave[joint_idx].outputs));
  }

  // Verify slave names and read encoder resolutions
  for (std::size_t joint_idx = 1; joint_idx <= kNumJoints; ++joint_idx) {
    if (std::strcmp(ec_slave[joint_idx].name, kExpectedSlaveName) != 0) {
      ec_close();
      return std::unexpected(
          Error{ErrorCode::kEtherCATError,
                std::string("Expected slave SOMANET at position ") +
                    std::to_string(joint_idx) + " but got '" +
                    ec_slave[joint_idx].name + "'"});
    }

    std::uint8_t encoder_source;
    int size = sizeof(encoder_source);
    ec_SDOread(joint_idx, 0x2012, 0x09, false, &size, &encoder_source,
               EC_TIMEOUTRXM);

    std::uint32_t encoder_resolution;
    size = sizeof(encoder_resolution);
    if (encoder_source == 1) {
      ec_SDOread(joint_idx, 0x2110, 0x03, false, &size, &encoder_resolution,
                 EC_TIMEOUTRXM);
    } else if (encoder_source == 2) {
      ec_SDOread(joint_idx, 0x2112, 0x03, false, &size, &encoder_resolution,
                 EC_TIMEOUTRXM);
    } else {
      ec_close();
      return std::unexpected(Error{
          ErrorCode::kEtherCATError,
          "No encoder configured for joint " + std::to_string(joint_idx)});
    }
    encoder_resolutions_[joint_idx - 1] = encoder_resolution;
  }

  // Read rated torque for each joint
  for (std::size_t joint_idx = 1; joint_idx <= kNumJoints; ++joint_idx) {
    auto result = ReadSDOValue(static_cast<std::uint16_t>(joint_idx), 0x6076,
                               0x00);
    if (!result) {
      return std::unexpected(
          Error{ErrorCode::kEtherCATError,
                "Failed to read rated torque for joint " +
                    std::to_string(joint_idx)});
    }
    rated_torques_[joint_idx - 1] = static_cast<float>(*result) / 1000.0f;
  }

  // Initialize joint admittances (wrist roll only)
  joint_admittances_.resize(kNumJoints);
  joint_admittances_.at(kWristRollIdx).emplace();
  if (!joint_admittances_.at(kWristRollIdx)->Init(10.0f, 500.0f)) {
    joint_admittances_.at(kWristRollIdx).reset();
    return std::unexpected(Error{ErrorCode::kEtherCATError,
                                 "Failed to init wrist roll admittance"});
  }

  // Start the EtherCAT error monitoring thread
  ecat_error_thread_ = std::jthread([this](std::stop_token st) {
    EcatCheck(st);
  });

  initialized_ = true;
  spdlog::info("ArmInterface initialized successfully");
  return {};
}

std::expected<void, Error> ArmInterface::StartControlLoop(
    float /*control_rate_hz*/) {
  if (!initialized_) {
    return std::unexpected(
        Error{ErrorCode::kNotInitialized, "Not initialized"});
  }
  if (control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopAlreadyRunning,
                                 "Control loop already running"});
  }

  control_loop_running_ = true;
  pdo_exchange_count_.store(0);

  // Start the EtherCAT control loop
  control_thread_ = std::jthread([this](std::stop_token st) {
    ControlLoop(st);
  });

  // Start the dynamic simulation thread
  dynamic_sim_thread_ = std::jthread([this](std::stop_token st) {
    bool ok = dynamic_simulator_.Run(st);
    if (!ok) {
      spdlog::error("Dynamic simulator exited with error");
    }
    dynamic_sim_exited_ = true;
  });

  spdlog::info("Control loop started");
  return {};
}

std::expected<void, Error> ArmInterface::StopControlLoop() {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  // Apply brakes on all joints before stopping
  {
    std::lock_guard<std::mutex> lock(hw_state_mtx_);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      Stop(out_somanet_, true, i);
      control_level_[i] = ControlLevel::kUndefined;
    }
  }

  // Request stop on both threads; jthread destructor handles join
  control_thread_.request_stop();
  dynamic_sim_thread_.request_stop();

  if (control_thread_.joinable()) control_thread_.join();
  if (dynamic_sim_thread_.joinable()) dynamic_sim_thread_.join();

  control_loop_running_ = false;
  in_normal_op_mode_ = false;
  spdlog::info("Control loop stopped");
  return {};
}

// ============================================================================
// Control mode switching
// ============================================================================

std::expected<void, Error> ArmInterface::SwitchControlMode(ControlLevel mode) {
  std::vector<ControlLevel> modes(kNumJoints, mode);
  return SwitchControlModeImpl(modes);
}

std::expected<void, Error> ArmInterface::SwitchControlMode(
    const std::vector<ControlLevel>& per_joint_modes) {
  if (per_joint_modes.size() != kNumJoints) {
    return std::unexpected(
        Error{ErrorCode::kInvalidArgument,
              "Expected " + std::to_string(kNumJoints) + " modes, got " +
                  std::to_string(per_joint_modes.size())});
  }
  return SwitchControlModeImpl(per_joint_modes);
}

std::expected<void, Error> ArmInterface::SwitchControlModeImpl(
    const std::vector<ControlLevel>& new_modes) {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  bool requested_quick_stop =
      std::any_of(new_modes.begin(), new_modes.end(),
                  [](ControlLevel m) { return m == ControlLevel::kQuickStop; });

  if (!allow_mode_change_ && !requested_quick_stop) {
    return std::unexpected(
        Error{ErrorCode::kModeChangeDenied,
              "Control mode change is disallowed at this moment"});
  }

  // RAII guard for mode_switch_in_progress_
  struct ModeSwitchGuard {
    std::atomic<bool>& flag;
    ModeSwitchGuard(std::atomic<bool>& f) : flag(f) { flag = true; }
    ~ModeSwitchGuard() { flag = false; }
  };
  ModeSwitchGuard guard(mode_switch_in_progress_);

  // Stop all joints with brakes
  {
    std::lock_guard<std::mutex> lock(hw_state_mtx_);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      Stop(out_somanet_, true, i);
    }
    ResetSpringAdjustState();
  }

  // Reset filters for hand-guided mode
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (new_modes[i] == ControlLevel::kHandGuided) {
      if (i == kWristPitchIdx) {
        hand_guided_pitch_brake_state_ = true;
        wrist_pitch_dial_filter_.Reset();
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

      // Set commutation offset state bit to Valid for torque control
      const std::int16_t valid_offset_state = 2;
      int result;
      {
        std::lock_guard<std::mutex> lock(hw_state_mtx_);
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

  // Clear commands and set new modes
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    threadsafe_commands_positions_[i] =
        std::numeric_limits<float>::quiet_NaN();
    threadsafe_commands_velocities_[i] = 0.0f;
    threadsafe_commands_efforts_[i] =
        std::numeric_limits<float>::quiet_NaN();
    threadsafe_commands_spring_adjust_[i] =
        std::numeric_limits<float>::quiet_NaN();
  }

  // Handle spring adjust mode
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (new_modes[i] == ControlLevel::kSpringAdjust) {
      allow_mode_change_ = false;
    }
  }

  {
    std::lock_guard<std::mutex> lock(hw_state_mtx_);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      control_level_[i] = new_modes[i];
    }
  }

  // One control cycle of delay
  osal_usleep(kCyclicLoopSleepUs);

  // Release brakes for non-undefined modes
  {
    std::lock_guard<std::mutex> lock(hw_state_mtx_);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (control_level_[i] != ControlLevel::kUndefined &&
          control_level_[i] != ControlLevel::kQuickStop) {
        out_somanet_[i]->Controlword = kNormalOpBrakesOff;
      }
    }
  }

  require_new_command_mode_ = false;
  return {};
}

// ============================================================================
// Streaming commands
// ============================================================================

std::expected<void, Error> ArmInterface::SetPositionCommand(
    const std::vector<float>& positions,
    std::function<bool()> halt_condition) {
  if (positions.size() != kNumJoints) {
    return std::unexpected(
        Error{ErrorCode::kInvalidArgument,
              "Expected " + std::to_string(kNumJoints) + " values"});
  }
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  // Switch to position mode if not already
  bool need_switch = false;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (control_level_[i] != ControlLevel::kPosition) {
      need_switch = true;
      break;
    }
  }
  if (need_switch) {
    auto result = SwitchControlMode(ControlLevel::kPosition);
    if (!result) return result;
  }

  {
    std::lock_guard<std::mutex> lock(halt_mutex_);
    halt_condition_ = std::move(halt_condition);
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    float mech_red = mechanical_reductions_[i].load();
    std::uint32_t enc_res = encoder_resolutions_[i].load();
    threadsafe_commands_positions_[i] = static_cast<float>(
        OutputShaftRadToInputTicks(positions[i], mech_red, enc_res, i));
  }
  return {};
}

std::expected<void, Error> ArmInterface::SetVelocityCommand(
    const std::vector<float>& velocities,
    std::function<bool()> halt_condition) {
  if (velocities.size() != kNumJoints) {
    return std::unexpected(
        Error{ErrorCode::kInvalidArgument,
              "Expected " + std::to_string(kNumJoints) + " values"});
  }
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  bool need_switch = false;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (control_level_[i] != ControlLevel::kVelocity) {
      need_switch = true;
      break;
    }
  }
  if (need_switch) {
    auto result = SwitchControlMode(ControlLevel::kVelocity);
    if (!result) return result;
  }

  {
    std::lock_guard<std::mutex> lock(halt_mutex_);
    halt_condition_ = std::move(halt_condition);
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    float mech_red = mechanical_reductions_[i].load();
    std::uint32_t enc_res = encoder_resolutions_[i].load();
    threadsafe_commands_velocities_[i] = static_cast<float>(
        OutputShaftRadPerSToInputTicksPerS(velocities[i], mech_red, enc_res));
  }
  return {};
}

std::expected<void, Error> ArmInterface::SetTorqueCommand(
    const std::vector<float>& torques,
    std::function<bool()> halt_condition) {
  if (torques.size() != kNumJoints) {
    return std::unexpected(
        Error{ErrorCode::kInvalidArgument,
              "Expected " + std::to_string(kNumJoints) + " values"});
  }
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  bool need_switch = false;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (control_level_[i] != ControlLevel::kTorque) {
      need_switch = true;
      break;
    }
  }
  if (need_switch) {
    auto result = SwitchControlMode(ControlLevel::kTorque);
    if (!result) return result;
  }

  {
    std::lock_guard<std::mutex> lock(halt_mutex_);
    halt_condition_ = std::move(halt_condition);
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    threadsafe_commands_efforts_[i] = std::clamp(torques[i], -1000.0f, 1000.0f);
  }
  return {};
}

// ============================================================================
// Generic commands
// ============================================================================

std::expected<void, Error> ArmInterface::SendCommand(
    const std::vector<float>& joint_commands) {
  if (joint_commands.size() != kNumJoints) {
    return std::unexpected(
        Error{ErrorCode::kInvalidArgument,
              "Expected " + std::to_string(kNumJoints) +
                  " commands, got " + std::to_string(joint_commands.size())});
  }
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    float mech_red = mechanical_reductions_[i].load();
    std::uint32_t enc_res = encoder_resolutions_[i].load();

    switch (control_level_[i]) {
      case ControlLevel::kPosition:
        threadsafe_commands_positions_[i] = static_cast<float>(
            OutputShaftRadToInputTicks(joint_commands[i], mech_red, enc_res, i));
        break;
      case ControlLevel::kVelocity:
        threadsafe_commands_velocities_[i] = static_cast<float>(
            OutputShaftRadPerSToInputTicksPerS(joint_commands[i], mech_red,
                                               enc_res));
        break;
      case ControlLevel::kTorque:
        threadsafe_commands_efforts_[i] =
            std::clamp(joint_commands[i], -1000.0f, 1000.0f);
        break;
      case ControlLevel::kSpringAdjust:
        threadsafe_commands_spring_adjust_[i] = joint_commands[i];
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
    auto i = static_cast<std::size_t>(joints[j]);
    if (i >= kNumJoints) {
      return std::unexpected(
          Error{ErrorCode::kInvalidArgument, "Invalid joint index"});
    }
    float mech_red = mechanical_reductions_[i].load();
    std::uint32_t enc_res = encoder_resolutions_[i].load();

    switch (control_level_[i]) {
      case ControlLevel::kPosition:
        threadsafe_commands_positions_[i] = static_cast<float>(
            OutputShaftRadToInputTicks(joint_commands[j], mech_red, enc_res, i));
        break;
      case ControlLevel::kVelocity:
        threadsafe_commands_velocities_[i] = static_cast<float>(
            OutputShaftRadPerSToInputTicksPerS(joint_commands[j], mech_red,
                                               enc_res));
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
// Trajectory (placeholder)
// ============================================================================

std::expected<void, Error> ArmInterface::SetTrajectory(
    const std::vector<float>& /*positions*/,
    const std::vector<float>& /*time_from_start*/) {
  // TODO: implement trajectory interpolation
  return std::unexpected(
      Error{ErrorCode::kInvalidMode, "Trajectory control not yet implemented"});
}

// ============================================================================
// Spring
// ============================================================================

std::expected<void, Error> ArmInterface::SetSpringSetpoint(
    float load_in_newtons) {
  // Convert load in Newtons to potentiometer ticks using the linear
  // relationship from SpringPotTicksToPayloadKg (inverted).
  // payload_kg = 0.03 * ticks - 5.8  =>  ticks = (payload_kg + 5.8) / 0.03
  float payload_kg = load_in_newtons / 9.81f;
  float target_ticks = (payload_kg + 5.8f) / 0.03f;
  spring_setpoint_target_ = target_ticks;
  return {};
}

// ============================================================================
// State queries
// ============================================================================

std::expected<std::vector<float>, Error> ArmInterface::GetPositions() const {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  std::lock_guard<std::mutex> lock(hw_state_mtx_);
  return state_positions_;
}

std::expected<std::vector<float>, Error> ArmInterface::GetVelocities() const {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  std::lock_guard<std::mutex> lock(hw_state_mtx_);
  return state_velocities_;
}

std::expected<std::vector<float>, Error> ArmInterface::GetTorques() const {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  std::lock_guard<std::mutex> lock(hw_state_mtx_);
  return state_torques_;
}

std::expected<std::vector<float>, Error> ArmInterface::GetAccelerations()
    const {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  std::lock_guard<std::mutex> lock(hw_state_mtx_);
  return state_accelerations_;
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
  // Read dial values via SDO (must be done from control thread, so return
  // cached values; for now return 0 as placeholder since SDO reads are
  // only safe from the control thread)
  return std::vector<int>{0, 0};
}

// ============================================================================
// Joint limits
// ============================================================================

std::expected<std::vector<JointLimitsInfo>, Error>
ArmInterface::GetJointLimits() const {
  std::vector<JointLimitsInfo> limits(kNumJoints);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    limits[i].has_limits =
        (i < has_position_limits_.size()) ? has_position_limits_[i] : false;
    limits[i].min_position =
        (i < min_position_limits_.size()) ? min_position_limits_[i] : 0.0f;
    limits[i].max_position =
        (i < max_position_limits_.size()) ? max_position_limits_[i] : 0.0f;
  }
  return limits;
}

// ============================================================================
// Internal helpers
// ============================================================================

void ArmInterface::WaitForGoodProcessData() {
  int wkc = -1;
  ec_send_processdata();
  wkc = ec_receive_processdata(EC_TIMEOUTRET);
  while (wkc < 0) {
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    osal_usleep(100000);
  }
}

void ArmInterface::HandleStartupFaults() {
  ec_readstate();
  for (int slave = 1; slave <= ec_slavecount; slave++) {
    if (std::strcmp(ec_slave[slave].name, kExpectedSlaveName) != 0) continue;
    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
      spdlog::warn("Slave {} is in SAFE_OP + ERROR, attempting ack", slave);
      ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
      ec_writestate(slave);
      osal_usleep(100000);
    }
  }
}

void ArmInterface::ResetSpringAdjustState() {
  spring_adjust_state_.time_prev = std::chrono::steady_clock::now();
  spring_adjust_state_.error_prev = std::nullopt;
}

bool ArmInterface::EStopEngaged() {
  ec_send_processdata();
  wkc_ = ec_receive_processdata(EC_TIMEOUTRET);

  if (wkc_ < expected_wkc_) {
    spdlog::error("Process data communication failed");
    return true;
  }

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
      in_somanet->PositionValue, mechanical_reductions_[kWristPitchIdx],
      encoder_resolutions_[kWristPitchIdx], kWristPitchIdx);
  float cosine_term = kWristPitchHoldTorque * std::cos(current_position);
  float drake_ff = kTorqueNmToPerMilleMultiplier *
                   TorqueNmToTorquePerMille(
                       dynamic_sim_state_->output.tau_required.at(kWristPitchIdx),
                       mechanical_reductions_[kWristPitchIdx],
                       rated_torques_[kWristPitchIdx]);
  out_somanet->TorqueOffset = static_cast<std::int16_t>(cosine_term + drake_ff);
}

void ArmInterface::ApplyFrictionCompensation(std::size_t joint_idx) {
  float joint_velocity_rad_s = InputTicksVelocityToOutputShaftRadPerS(
      in_somanet_[joint_idx]->VelocityValue,
      mechanical_reductions_[joint_idx], encoder_resolutions_[joint_idx]);

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
      mechanical_reductions_[kYaw2Idx], encoder_resolutions_[kYaw2Idx],
      kYaw2Idx);

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

float ArmInterface::SpringAdjustByLIPS(float target_position,
                                       std::int32_t current_lips_position,
                                       bool& allow_mode_change) {
  constexpr float kP = 15.0f;
  constexpr float kD = 0.0f;
  float error = target_position - static_cast<float>(current_lips_position);
  auto time_now = std::chrono::steady_clock::now();
  std::chrono::duration<float> time_elapsed =
      time_now - spring_adjust_state_.time_prev;
  float error_dt = 0.0f;
  if (spring_adjust_state_.error_prev) {
    error_dt =
        (error - *spring_adjust_state_.error_prev) / time_elapsed.count();
  }
  spring_adjust_state_.error_prev = error;
  spring_adjust_state_.time_prev = time_now;
  float actuator_torque = kP * error + kD * error_dt;

  if (actuator_torque > 0) {
    actuator_torque = std::clamp(actuator_torque, kSpringAdjustMinTorque,
                                 kSpringAdjustMaxTorque);
  } else {
    actuator_torque = std::clamp(actuator_torque, -kSpringAdjustMaxTorque,
                                 -kSpringAdjustMinTorque);
  }

  if (std::abs(error) < 50.0f) {
    actuator_torque = 0.0f;
    if (!allow_mode_change) {
      allow_mode_change = true;
    }
  }

  return actuator_torque;
}

// ============================================================================
// EtherCAT error monitoring
// ============================================================================

void ArmInterface::EcatCheck(std::stop_token stop_token) {
  std::uint8_t currentgroup = 0;

  while (!stop_token.stop_requested()) {
    if (in_normal_op_mode_ &&
        ((wkc_ < expected_wkc_) || ec_group[currentgroup].docheckstate)) {
      ec_group[currentgroup].docheckstate = false;
      ec_readstate();
      for (int slave = 1; slave <= ec_slavecount; slave++) {
        if ((ec_slave[slave].group == currentgroup) &&
            (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
          ec_group[currentgroup].docheckstate = true;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            spdlog::error("Slave {} in SAFE_OP+ERROR, attempting ack", slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
            spdlog::warn("Slave {} in SAFE_OP, changing to OPERATIONAL",
                         slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          } else if (ec_slave[slave].state > EC_STATE_NONE) {
            if (ec_reconfig_slave(slave, kEcTimeoutMon)) {
              ec_slave[slave].islost = false;
              spdlog::info("Slave {} reconfigured", slave);
            }
          } else if (!ec_slave[slave].islost) {
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE) {
              ec_slave[slave].islost = true;
              spdlog::error("Slave {} lost", slave);
            }
          }

          if (ec_slave[slave].islost) {
            if (ec_slave[slave].state == EC_STATE_NONE) {
              if (ec_recover_slave(slave, kEcTimeoutMon)) {
                ec_slave[slave].islost = false;
                spdlog::info("Slave {} recovered", slave);
              }
            } else {
              ec_slave[slave].islost = false;
              spdlog::info("Slave {} found", slave);
            }
          }
        }
      }
    }
    osal_usleep(10000);
  }
}

// ============================================================================
// Main control loop
// ============================================================================

void ArmInterface::ControlLoop(std::stop_token stop_token) {
  std::vector<bool> first_iteration(kNumJoints, true);

  while (!stop_token.stop_requested() && !dynamic_sim_exited_) {
    const bool estop = EStopEngaged();
    if (estop) {
      require_new_command_mode_ = true;
      // Match SynapticonSystemInterface::SomanetCyclicLoop: count exchanges when
      // e-stop path still has valid PDO (EStopEngaged already ran send/receive).
      if (wkc_.load() >= expected_wkc_.load() &&
          pdo_exchange_count_.load() < kMinPdoExchanges) {
        pdo_exchange_count_.fetch_add(1);
      }
    } else if (pdo_exchange_count_.load() < kMinPdoExchanges) {
      pdo_exchange_count_.fetch_add(1);
    }

    // After enough PDO exchanges, update state readout so GetPositions() etc.
    // return settled data (see synapticon read() MIN_PDO_EXCHANGES gate).
    if (pdo_exchange_count_.load() >= kMinPdoExchanges) {
      std::lock_guard<std::mutex> lock(hw_state_mtx_);
      for (std::size_t joint_idx = 0; joint_idx < kNumJoints; ++joint_idx) {
        state_velocities_[joint_idx] = InputTicksVelocityToOutputShaftRadPerS(
            in_somanet_[joint_idx]->VelocityValue,
            mechanical_reductions_[joint_idx],
            encoder_resolutions_[joint_idx]);
        state_positions_[joint_idx] = InputTicksToOutputShaftRad(
            in_somanet_[joint_idx]->PositionValue,
            mechanical_reductions_[joint_idx],
            encoder_resolutions_[joint_idx], joint_idx);
        state_torques_[joint_idx] =
            (in_somanet_[joint_idx]->TorqueValue / 1000.0f) *
            rated_torques_[joint_idx].load() *
            mechanical_reductions_[joint_idx].load();
        state_accelerations_[joint_idx] = 0.0f;
      }
    }

    if (estop) {
      osal_usleep(kCyclicLoopSleepUs);
      continue;
    }

    // Check halt condition
    {
      std::lock_guard<std::mutex> lock(halt_mutex_);
      if (halt_condition_ && halt_condition_()) {
        spdlog::info("Halt condition triggered, stopping all joints");
        std::lock_guard<std::mutex> hw_lock(hw_state_mtx_);
        for (std::size_t i = 0; i < kNumJoints; ++i) {
          Stop(out_somanet_, true, i);
        }
        halt_condition_ = nullptr;
        osal_usleep(kCyclicLoopSleepUs);
        continue;
      }
    }

    {
      std::lock_guard<std::mutex> lock(hw_state_mtx_);

      // Read GPIO via SDO
      auto wr_roll_gpio = ReadSDOValue(
          static_cast<std::uint16_t>(kWristRollIdx + 1), 0x60FD, 0x00);
      if (!wr_roll_gpio) {
        spdlog::error("ReadSDOValue() failed for GPIO");
        osal_usleep(kCyclicLoopSleepUs);
        continue;
      }
      const std::int32_t lips_spring_position =
          in_somanet_[kSpringAdjustIdx]->PositionValue;

      hw_function_enable_ =
          static_cast<float>((*wr_roll_gpio & (1 << 16)) >> 16);
      hw_decomp_button_ =
          static_cast<float>((*wr_roll_gpio & (1 << 17)) >> 17);
      hw_comp_button_ =
          static_cast<float>((*wr_roll_gpio & (1 << 18)) >> 18);
      const bool admittance_button_pressed = (*wr_roll_gpio & (1 << 19)) >> 19;

      const bool deadman_pressed = hw_function_enable_.load() > 0.5f;
      static bool prev_deadman_pressed = false;

      // Check if deadman pressed initially (stuck button safety)
      if (first_iteration[0] && deadman_pressed) {
        first_iteration[0] = false;
        allow_mode_change_ = false;
        prev_deadman_pressed = true;
        spdlog::error(
            "Function enable pressed at startup - blocking hand_guided");
        osal_usleep(kCyclicLoopSleepUs);
        continue;
      }
      first_iteration[0] = false;

      if (require_new_command_mode_) {
        allow_mode_change_ = true;
        osal_usleep(kCyclicLoopSleepUs);
        continue;
      }

      // Update payload mass from spring potentiometer
      float payload_mass_kg = SpringPotTicksToPayloadKg(lips_spring_position);
      if (payload_mass_kg > 0) {
        dynamic_sim_state_->input.payload_mass = payload_mass_kg;
      } else {
        dynamic_sim_state_->input.payload_mass = 0.0f;
      }

      // Deadman mode change state tracking
      if (control_level_[0] == ControlLevel::kHandGuided &&
          deadman_pressed && !prev_deadman_pressed) {
        allow_mode_change_ = false;
        prev_deadman_pressed = true;
      } else if (prev_deadman_pressed && !deadman_pressed) {
        allow_mode_change_ = true;
        prev_deadman_pressed = false;
      }

      // Run state machine for each joint
      for (std::size_t joint_idx = 0; joint_idx < kNumJoints; ++joint_idx) {
        if (first_iteration.at(joint_idx)) {
          out_somanet_[joint_idx]->OpMode = kProfileTorqueMode;
          first_iteration.at(joint_idx) = false;
        }

        // Gravity compensation
        ApplyGravComp(joint_idx, in_somanet_[joint_idx],
                      out_somanet_[joint_idx]);

        StateMachineStep(joint_idx, lips_spring_position,
                         admittance_button_pressed);

        StatusWordErrorLogging(joint_idx);
      }
    }  // mutex scope

    osal_usleep(kCyclicLoopSleepUs);
  }
}

// ============================================================================
// State machine
// ============================================================================

void ArmInterface::StateMachineStep(std::size_t joint_idx,
                                    std::int32_t lips_spring_position,
                                    bool admittance_button_pressed) {
  const auto statusword = in_somanet_[joint_idx]->Statusword;

  // Fault
  if ((statusword & 0x004F) == 0x0008) {
    HandleFault(in_somanet_[joint_idx], out_somanet_[joint_idx], joint_idx);
    return;
  }
  // Switch on disabled
  if ((statusword & 0x004F) == 0x0040) {
    HandleShutdown(out_somanet_, joint_idx, control_level_,
                   mode_switch_in_progress_);
    return;
  }
  // Ready to switch on
  if ((statusword & 0x006F) == 0x0021) {
    HandleSwitchOn(out_somanet_, joint_idx);
    return;
  }
  // Switched on
  if ((statusword & 0x006F) == 0x0023) {
    HandleEnableOperation(out_somanet_, joint_idx, mode_switch_in_progress_);
    return;
  }
  // Quick stop active
  if ((statusword & 0x006F) == 0x0007) {
    if (control_level_[joint_idx] != ControlLevel::kQuickStop) {
      out_somanet_[joint_idx]->Controlword = 0b00000000;
    }
    return;
  }
  // Fault reaction active or Not ready to switch on
  if ((statusword & 0x004F) == 0x000F ||
      (statusword & 0x004F) == 0x0000) {
    return;
  }

  // ---- Normal operation (Operation Enabled) ----
  if ((statusword & 0x0027) != 0x0027) {
    spdlog::warn("Joint {} in unrecognized Statusword state: 0x{:04X}",
                 joint_idx, statusword);
    return;
  }

  in_normal_op_mode_ = true;
  const bool mode_switch = mode_switch_in_progress_;

  // --- Hand-guided ---
  if (control_level_[joint_idx] == ControlLevel::kHandGuided) {
    if ((joint_idx != kSpringAdjustIdx) &&
        (joint_idx != kElevationInertialIdx)) {
      // Reset admittance if button not pressed
      if (!admittance_button_pressed && joint_admittances_[joint_idx]) {
        joint_admittances_[joint_idx]->Reset();
      }

      // Admittance mode
      if (admittance_button_pressed && joint_admittances_[joint_idx]) {
        float joint_vel = 0.0f;
        if (!std::isnan(
                dynamic_sim_state_->output.tau_required.at(joint_idx)) &&
            !std::isnan(dynamic_sim_state_->output.estimated_accelerations
                            [joint_idx])) {
          float T_ext = in_somanet_[joint_idx]->TorqueValue -
                        dynamic_sim_state_->output.tau_required.at(joint_idx);
          float joint_accel =
              dynamic_sim_state_->output.estimated_accelerations[joint_idx];
          joint_vel = -kMysteryVelocityMultiplier *
                      mechanical_reductions_[joint_idx].load() *
                      joint_admittances_[joint_idx]->CalculateVelocity(
                          joint_accel, T_ext);
        }
        SetVelocityWithLimits(joint_idx, in_somanet_[joint_idx],
                              has_position_limits_, min_position_limits_,
                              max_position_limits_,
                              mechanical_reductions_[joint_idx],
                              encoder_resolutions_[joint_idx], joint_vel,
                              out_somanet_[joint_idx]);
        out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
      }
      // Wrist pitch (dial controlled)
      else if (joint_idx == kWristPitchIdx) {
        auto wrist_pitch_dial_value = ReadSDOValue(
            static_cast<std::uint16_t>(kWristRollIdx + 1), 0x2402, 0x00);
        if (!wrist_pitch_dial_value) {
          wrist_pitch_dial_value =
              elevate_config_.button_config.pitch_dial.center;
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
        if (std::abs(normalized_dial) < kWristPitchDeadband) {
          normalized_dial = 0.0f;
        }

        float velocity = 0.0f;
        float v_max = kMysteryVelocityMultiplier *
                      mechanical_reductions_[joint_idx].load() *
                      kMaxWristPitchVelocity;
        float slope = v_max / (1.0f - kWristPitchDeadband);
        if (normalized_dial > 1.0f) {
          velocity = v_max;
        } else if (normalized_dial >= kWristPitchDeadband) {
          velocity = slope * (normalized_dial - kWristPitchDeadband);
        } else if (normalized_dial < -1.0f) {
          velocity = -v_max;
        } else if (normalized_dial <= -kWristPitchDeadband) {
          velocity = slope * (normalized_dial + 1.0f) - v_max;
        }
        velocity = wrist_pitch_dial_filter_.Filter(velocity);

        if (hand_guided_pitch_brake_state_ &&
            std::abs(normalized_dial) > kWristPitchBrakeOffThreshold) {
          if (!mode_switch) {
            out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
          }
          hand_guided_pitch_brake_state_ = false;
        }

        if (!hand_guided_pitch_brake_state_) {
          SetVelocityWithLimits(joint_idx, in_somanet_[joint_idx],
                                has_position_limits_, min_position_limits_,
                                max_position_limits_,
                                mechanical_reductions_[joint_idx],
                                encoder_resolutions_[joint_idx], velocity,
                                out_somanet_[joint_idx]);
        }
      }
      // Wrist roll (dial controlled)
      else if (joint_idx == kWristRollIdx) {
        auto wrist_roll_dial_value = ReadSDOValue(
            static_cast<std::uint16_t>(kWristYawIdx + 1), 0x2402, 0x00);
        if (!wrist_roll_dial_value) {
          wrist_roll_dial_value =
              elevate_config_.button_config.roll_dial.center;
        }

        *wrist_roll_dial_value = std::clamp(
            *wrist_roll_dial_value,
            elevate_config_.button_config.roll_dial.min,
            elevate_config_.button_config.roll_dial.max);
        float normalized_dial =
            -static_cast<float>(
                *wrist_roll_dial_value -
                elevate_config_.button_config.roll_dial.center) /
            (0.5f * (elevate_config_.button_config.roll_dial.max -
                     elevate_config_.button_config.roll_dial.min));
        if (std::abs(normalized_dial) < kWristRollDeadband) {
          normalized_dial = 0.0f;
        }

        float velocity = 0.0f;
        float v_max = kMysteryVelocityMultiplier *
                      mechanical_reductions_[joint_idx].load() *
                      kMaxWristRollVelocity;
        float slope = v_max / (1.0f - kWristRollDeadband);
        if (normalized_dial > 1.0f) {
          velocity = v_max;
        } else if (normalized_dial >= kWristRollDeadband) {
          velocity = slope * (normalized_dial - kWristRollDeadband);
        } else if (normalized_dial < -1.0f) {
          velocity = -v_max;
        } else if (normalized_dial <= -kWristRollDeadband) {
          velocity = slope * (normalized_dial + 1.0f) - v_max;
        }

        out_somanet_[joint_idx]->TargetVelocity =
            static_cast<std::int32_t>(velocity);
        out_somanet_[joint_idx]->OpMode = kCyclicVelocityMode;
        out_somanet_[joint_idx]->VelocityOffset = 0;
        if (!mode_switch) {
          out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
        }
      }
      // Remaining joints (yaw1, yaw2, wrist_yaw): torque mode
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
            mechanical_reductions_[joint_idx],
            encoder_resolutions_[joint_idx], joint_idx);
        auto result = JointLimitCmdClamp(
            joint_idx, ControlLevel::kTorque, current_position,
            has_position_limits_, min_position_limits_, max_position_limits_,
            torque);
        if (result.has_value()) {
          torque = result->second;
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
    // Elevation link joints (spring_adjust, elevation_inertial)
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
  }
  // --- Velocity ---
  else if (control_level_[joint_idx] == ControlLevel::kVelocity) {
    if (!std::isnan(threadsafe_commands_velocities_[joint_idx])) {
      out_somanet_[joint_idx]->TargetVelocity =
          static_cast<std::int32_t>(threadsafe_commands_velocities_[joint_idx].load());
      out_somanet_[joint_idx]->OpMode = kCyclicVelocityMode;
      out_somanet_[joint_idx]->VelocityOffset = 0;
      if (!mode_switch) {
        out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
      }
    }
  }
  // --- Position ---
  else if (control_level_[joint_idx] == ControlLevel::kPosition) {
    if (!std::isnan(threadsafe_commands_positions_[joint_idx])) {
      out_somanet_[joint_idx]->TargetPosition =
          static_cast<std::int32_t>(threadsafe_commands_positions_[joint_idx].load());
      out_somanet_[joint_idx]->OpMode = kCyclicPositionMode;
      out_somanet_[joint_idx]->VelocityOffset = 0;
      if (!mode_switch) {
        out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
      }
    }
  }
  // --- Quick stop ---
  else if (control_level_[joint_idx] == ControlLevel::kQuickStop) {
    Stop(out_somanet_, true, joint_idx);
  }
  // --- Spring adjust ---
  else if (control_level_[joint_idx] == ControlLevel::kSpringAdjust) {
    if (joint_idx == kSpringAdjustIdx) {
      if (!allow_mode_change_) {
        bool allow = allow_mode_change_.load();
        float target = spring_setpoint_target_.load();
        if (target == 0.0f) {
          target = static_cast<float>(
              elevate_config_.spring_setpoints.spring_setpoint_unloaded);
        }
        float actuator_torque =
            SpringAdjustByLIPS(target, lips_spring_position, allow);
        allow_mode_change_ = allow;

        out_somanet_[joint_idx]->TargetTorque =
            static_cast<std::int16_t>(actuator_torque);
        out_somanet_[joint_idx]->OpMode = kProfileTorqueMode;
        out_somanet_[joint_idx]->TorqueOffset = 0;
        if (!mode_switch) {
          out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
        }
      } else {
        control_level_[joint_idx] = ControlLevel::kQuickStop;
      }
    } else {
      // Non-spring joints in spring adjust mode: quick stop
      Stop(out_somanet_, true, joint_idx);
    }
  }
  // --- Torque ---
  else if (control_level_[joint_idx] == ControlLevel::kTorque) {
    if (!std::isnan(threadsafe_commands_efforts_[joint_idx])) {
      out_somanet_[joint_idx]->TargetTorque =
          static_cast<std::int16_t>(threadsafe_commands_efforts_[joint_idx].load());
      out_somanet_[joint_idx]->OpMode = kProfileTorqueMode;
      if (!mode_switch) {
        out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
      }
    }
  }
  // --- Undefined ---
  else if (control_level_[joint_idx] == ControlLevel::kUndefined) {
    out_somanet_[joint_idx]->OpMode = kProfileTorqueMode;
    out_somanet_[joint_idx]->TorqueOffset = 0;
    out_somanet_[joint_idx]->Controlword = kQuickStopCtrlWord;
  }
}

void ArmInterface::StatusWordErrorLogging(std::size_t joint_idx) {
  if (in_somanet_[joint_idx]->Statusword & (1 << 11)) {
    spdlog::warn(
        "Internal limit active for joint {}, TorqueDemand: {}, "
        "VelocityDemandValue: {}",
        joint_idx, in_somanet_[joint_idx]->TorqueDemand,
        in_somanet_[joint_idx]->VelocityDemandValue);
  }
}

}  // namespace elevated_control
