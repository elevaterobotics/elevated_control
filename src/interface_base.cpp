// Copyright (c) 2025 Elevate Robotics Inc

#include "elevated_control/interface_base.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <soem/ethercat.h>
#pragma GCC diagnostic pop

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "elevated_control/state_machine.hpp"
#include "elevated_control/unit_conversions.hpp"

using namespace std::chrono_literals;

namespace elevated_control {

namespace {

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

constexpr std::chrono::seconds kProcessDataWarnThrottleInterval{2};
constexpr int kEcTimeoutMon = 500;

}  // namespace

// SDO read helpers (declared in state_machine.hpp)
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

std::optional<std::string> ReadSDOString(std::uint16_t slave,
                                         std::uint16_t index,
                                         std::uint8_t subindex) {
  char buf[256] = {};
  int object_size = sizeof(buf) - 1;
  int result = ec_SDOread(slave, index, subindex, FALSE, &object_size,
                          buf, EC_TIMEOUTRXM);
  if (result <= 0) return std::nullopt;
  buf[object_size] = '\0';
  return std::string(buf, static_cast<std::size_t>(object_size));
}

std::optional<float> ReadDriveMechanicalReduction(std::uint16_t slave_idx) {
  const auto gear_ratio_num = ReadSDOValue(slave_idx, 0x6091, 0x01);
  const auto gear_ratio_den = ReadSDOValue(slave_idx, 0x6091, 0x02);
  if (!gear_ratio_num || !gear_ratio_den) {
    return std::nullopt;
  }
  if ((*gear_ratio_num <= 0) || (*gear_ratio_den <= 0)) {
    return std::nullopt;
  }
  return static_cast<float>(*gear_ratio_num) /
         static_cast<float>(*gear_ratio_den);
}

// ============================================================================
// Construction / Destruction
// ============================================================================

SynapticonBase::SynapticonBase(const SynapticonBaseConfig& config)
    : base_config_(config) {}

SynapticonBase::~SynapticonBase() {
  shutdown_requested_ = true;
  in_normal_op_mode_ = false;

  if (ecat_error_thread_.joinable()) {
    ecat_error_thread_.request_stop();
  }

  if (control_loop_running_) {
    auto result = StopControlLoop();
    if (!result &&
        result.error().code != ErrorCode::kControlLoopNotRunning) {
      spdlog::error("StopControlLoop failed in destructor: {}",
                    result.error().message);
    }
  }

  if (ecat_error_thread_.joinable()) {
    ecat_error_thread_.join();
  }

  if (initialized_) {
    {
      std::lock_guard<std::mutex> lock(ecat_mtx_);
      ec_slave[0].state = EC_STATE_INIT;
      ec_writestate(0);
    }
    std::this_thread::sleep_for(1000ms);
    std::lock_guard<std::mutex> lock(ecat_mtx_);
    ec_close();
  }
}

// ============================================================================
// Lifecycle
// ============================================================================

std::expected<void, Error> SynapticonBase::Initialize() {
  if (initialized_) {
    return std::unexpected(
        Error{ErrorCode::kAlreadyInitialized, "Already initialized"});
  }

  shutdown_requested_ = false;

  if (!InterfaceExists(base_config_.network_interface.c_str())) {
    return std::unexpected(Error{
        ErrorCode::kEtherCATError,
        "Network interface '" + base_config_.network_interface + "' does not exist"});
  }

  int ec_init_status = ec_init(base_config_.network_interface.c_str());
  if (ec_init_status <= 0) {
    ec_close();
    return std::unexpected(Error{ErrorCode::kEtherCATError,
                                 "EtherCAT init failed on " +
                                     base_config_.network_interface});
  }

  if (ec_config_init(false) <= 0) {
    ec_close();
    return std::unexpected(
        Error{ErrorCode::kEtherCATError, "No EtherCAT slaves found"});
  }

  num_joints_ = static_cast<std::size_t>(ec_slavecount);
  if (base_config_.expected_slave_count > 0 &&
      num_joints_ != base_config_.expected_slave_count) {
    ec_close();
    return std::unexpected(Error{
        ErrorCode::kEtherCATError,
        "Expected " + std::to_string(base_config_.expected_slave_count) +
            " slaves but found " + std::to_string(num_joints_)});
  }

  // Resize all per-joint containers
  in_somanet_.resize(num_joints_, nullptr);
  in_somanet_snapshot_.resize(num_joints_);
  out_somanet_.resize(num_joints_, nullptr);
  control_mode_.resize(num_joints_, ControlMode::kUndefined);
  hold_in_shutdown_.clear();
  for (std::size_t i = 0; i < num_joints_; ++i) {
    hold_in_shutdown_.emplace_back(false);
  }

  state_positions_.assign(num_joints_, std::numeric_limits<float>::quiet_NaN());
  state_velocities_.assign(num_joints_, std::numeric_limits<float>::quiet_NaN());
  state_torques_.assign(num_joints_, std::numeric_limits<float>::quiet_NaN());
  state_accelerations_.assign(num_joints_, std::numeric_limits<float>::quiet_NaN());

  mechanical_reductions_.resize(num_joints_);
  position_reductions_.resize(num_joints_);
  si_velocity_units_.resize(num_joints_);
  encoder_resolutions_.resize(num_joints_);
  rated_torques_.resize(num_joints_);
  for (std::size_t i = 0; i < num_joints_; ++i) {
    mechanical_reductions_[i] = 1.0f;
    position_reductions_[i] = 1.0f;
    si_velocity_units_[i] = 0;
    encoder_resolutions_[i] = 1;
    rated_torques_[i] = 1.0f;
  }

  threadsafe_commands_positions_.resize(num_joints_);
  threadsafe_commands_velocities_.resize(num_joints_);
  threadsafe_commands_efforts_.resize(num_joints_);
  last_velocity_write_time_ns_.clear();
  for (std::size_t i = 0; i < num_joints_; ++i) {
    last_velocity_write_time_ns_.emplace_back(0);
  }
  for (std::size_t i = 0; i < num_joints_; ++i) {
    threadsafe_commands_positions_[i] =
        std::numeric_limits<float>::quiet_NaN();
    threadsafe_commands_velocities_[i] = 0.0f;
    threadsafe_commands_efforts_[i] =
        std::numeric_limits<float>::quiet_NaN();
    last_velocity_write_time_ns_[i].store(0, std::memory_order_relaxed);
  }

  startup_angle_wrap_flag_.clear();
  startup_angle_wrap_flag_.resize(num_joints_);
  startup_angle_wrap_value_.resize(num_joints_);
  for (auto& v : startup_angle_wrap_value_) v.store(0.0f);

  has_position_limits_.assign(num_joints_, false);
  min_position_limits_.assign(num_joints_, -std::numeric_limits<float>::max());
  max_position_limits_.assign(num_joints_, std::numeric_limits<float>::max());
  max_brake_torques_.assign(num_joints_, 250.0f);

  velocity_filters_.clear();
  velocity_filters_.reserve(num_joints_);
  for (std::size_t i = 0; i < num_joints_; ++i) {
    velocity_filters_.emplace_back(0.0f);
  }

  ec_config_map(&io_map_);
  ec_configdc();
  WaitForGoodProcessData();
  HandleStartupFaults();

  // Verify slaves and read per-joint parameters
  for (std::size_t joint_idx = 1; joint_idx <= num_joints_; ++joint_idx) {
    if (std::strcmp(ec_slave[joint_idx].name, kExpectedSlaveName.data()) != 0) {
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

    const auto si_velocity_unit =
        ReadSDOValue(static_cast<std::uint16_t>(joint_idx), 0x60A9, 0x00);
    if (!si_velocity_unit) {
      ec_close();
      return std::unexpected(Error{
          ErrorCode::kEtherCATError,
          "ReadSDOValue failed for 0x60A9 on joint " + std::to_string(joint_idx)});
    }
    si_velocity_units_[joint_idx - 1] = *si_velocity_unit;

    auto rated_result = ReadSDOValue(static_cast<std::uint16_t>(joint_idx),
                                     0x6076, 0x00);
    if (!rated_result) {
      ec_close();
      return std::unexpected(
          Error{ErrorCode::kEtherCATError,
                "Failed to read rated torque for joint " +
                    std::to_string(joint_idx)});
    }
    rated_torques_[joint_idx - 1] = static_cast<float>(*rated_result) / 1000.0f;

    const auto drive_reduction = ReadDriveMechanicalReduction(
        static_cast<std::uint16_t>(joint_idx));
    if (!drive_reduction) {
      ec_close();
      return std::unexpected(
          Error{ErrorCode::kEtherCATError,
                "Failed to read valid 0x6091 gear ratio for joint " +
                    std::to_string(joint_idx)});
    }
    mechanical_reductions_[joint_idx - 1] = *drive_reduction;
    position_reductions_[joint_idx - 1] =
        (encoder_source == 2) ? 1.0f : *drive_reduction;
    spdlog::info("Joint {}: mechanical reduction {:.6f}, encoder resolution {}",
                 joint_idx - 1, *drive_reduction, encoder_resolution);
    spdlog::debug(
        "Joint {} DEBUG: si_velocity_unit=0x{:08X} ({}), mechanical_reduction={:.6f}, "
        "position_reduction={:.6f}, encoder_resolution={}, rated_torque={:.4f}, "
        "encoder_source={}",
        joint_idx - 1, static_cast<std::uint32_t>(*si_velocity_unit),
        *si_velocity_unit, mechanical_reductions_[joint_idx - 1].load(),
        position_reductions_[joint_idx - 1].load(),
        encoder_resolutions_[joint_idx - 1].load(),
        rated_torques_[joint_idx - 1].load(), encoder_source);
  }

  // Request operational state
  expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  for (int slave_id = 0; slave_id < ec_slavecount; ++slave_id) {
    ec_slave[slave_id].state = EC_STATE_OPERATIONAL;
  }
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  ec_writestate(0);

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
  for (std::size_t joint_idx = 1; joint_idx <= num_joints_; ++joint_idx) {
    const std::size_t i = joint_idx - 1;
    in_somanet_[i] =
        reinterpret_cast<InSomanet50t*>(ec_slave[joint_idx].inputs);
    out_somanet_[i] =
        reinterpret_cast<OutSomanet50t*>(ec_slave[joint_idx].outputs);
  }

  ecat_error_thread_ = std::jthread([this](std::stop_token st) {
    EcatCheck(st);
  });

  initialized_ = true;
  spdlog::info("SynapticonBase initialized with {} joints", num_joints_);
  return {};
}

std::expected<void, Error> SynapticonBase::StartControlLoop(
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
  shutdown_requested_ = false;
  pdo_exchange_count_.store(0);

  control_thread_ = std::jthread([this](std::stop_token st) {
    ControlLoop(st);
  });

  spdlog::info("Control loop started");
  return {};
}

bool SynapticonBase::IsControlLoopReady() const noexcept {
  return control_loop_running_.load() &&
         pdo_exchange_count_.load() >= kMinPdoExchanges;
}

std::expected<void, Error> SynapticonBase::StopControlLoop() {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  {
    std::scoped_lock lock(ecat_mtx_, hw_state_mtx_);
    for (std::size_t i = 0; i < num_joints_; ++i) {
      Stop(out_somanet_[i], true);
      control_mode_[i] = ControlMode::kUndefined;
    }
  }

  control_thread_.request_stop();
  if (control_thread_.joinable()) control_thread_.join();

  control_loop_running_ = false;
  in_normal_op_mode_ = false;
  spdlog::info("Control loop stopped");
  return {};
}

// ============================================================================
// Control mode switching
// ============================================================================

std::expected<void, Error> SynapticonBase::SwitchControlMode(ControlMode mode) {
  std::vector<ControlMode> modes(num_joints_, mode);
  return SwitchControlModeImpl(modes);
}

std::expected<void, Error> SynapticonBase::SwitchControlMode(
    const std::vector<ControlMode>& per_joint_modes) {
  return SwitchControlModeImpl(per_joint_modes);
}

std::expected<void, Error> SynapticonBase::SwitchControlModeImpl(
    const std::vector<ControlMode>& new_modes) {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }

  bool requested_quick_stop =
      std::any_of(new_modes.begin(), new_modes.end(),
                  [](ControlMode m) { return m == ControlMode::kQuickStop; });

  if (!allow_mode_change_ && !requested_quick_stop) {
    return std::unexpected(
        Error{ErrorCode::kModeChangeDenied,
              "Control mode change is disallowed at this moment"});
  }

  struct ModeSwitchGuard {
    std::atomic<bool>& flag;
    ModeSwitchGuard(std::atomic<bool>& f) : flag(f) { flag = true; }
    ~ModeSwitchGuard() { flag = false; }
  };
  ModeSwitchGuard guard(mode_switch_in_progress_);

  {
    std::scoped_lock lock(ecat_mtx_, hw_state_mtx_);
    for (std::size_t i = 0; i < num_joints_; ++i) {
      Stop(out_somanet_[i], true);
    }
  }

  for (std::size_t i = 0; i < num_joints_; ++i) {
    threadsafe_commands_positions_[i] =
        std::numeric_limits<float>::quiet_NaN();
    threadsafe_commands_velocities_[i] = 0.0f;
    last_velocity_write_time_ns_[i].store(0, std::memory_order_relaxed);
    threadsafe_commands_efforts_[i] =
        std::numeric_limits<float>::quiet_NaN();
    hold_in_shutdown_[i] = false;
  }

  {
    std::lock_guard<std::mutex> lock(hw_state_mtx_);
    for (std::size_t i = 0; i < num_joints_; ++i) {
      control_mode_[i] = new_modes[i];
    }
  }

  osal_usleep(kCyclicLoopSleepUs);

  {
    std::scoped_lock lock(ecat_mtx_, hw_state_mtx_);
    for (std::size_t i = 0; i < num_joints_; ++i) {
      if (control_mode_[i] != ControlMode::kUndefined &&
          control_mode_[i] != ControlMode::kQuickStop) {
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

std::expected<void, Error> SynapticonBase::SetPositionCommand(
    const std::vector<float>& positions,
    std::function<bool()> halt_condition) {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  if (positions.size() != num_joints_) {
    return std::unexpected(Error{ErrorCode::kInvalidArgument,
                                 "positions size mismatch"});
  }

  bool need_switch = false;
  for (std::size_t i = 0; i < num_joints_; ++i) {
    if (control_mode_[i] != ControlMode::kPosition) {
      need_switch = true;
      break;
    }
  }
  if (need_switch) {
    auto result = SwitchControlMode(ControlMode::kPosition);
    if (!result) return result;
  }

  {
    std::lock_guard<std::mutex> lock(halt_mutex_);
    halt_condition_ = std::move(halt_condition);
  }

  for (std::size_t i = 0; i < num_joints_; ++i) {
    float pos_red = position_reductions_[i].load();
    std::uint32_t enc_res = encoder_resolutions_[i].load();
    threadsafe_commands_positions_[i] = static_cast<float>(
        OutputShaftRadToInputTicks(positions[i], pos_red, enc_res,
                                   startup_angle_wrap_value_[i]));
  }
  return {};
}

std::expected<void, Error> SynapticonBase::SetVelocityCommand(
    const std::vector<float>& velocities,
    std::function<bool()> halt_condition) {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  if (velocities.size() != num_joints_) {
    return std::unexpected(Error{ErrorCode::kInvalidArgument,
                                 "velocities size mismatch"});
  }

  bool need_switch = false;
  for (std::size_t i = 0; i < num_joints_; ++i) {
    if (control_mode_[i] != ControlMode::kVelocity) {
      need_switch = true;
      break;
    }
  }
  if (need_switch) {
    auto result = SwitchControlMode(ControlMode::kVelocity);
    if (!result) return result;
  }

  {
    std::lock_guard<std::mutex> lock(halt_mutex_);
    halt_condition_ = std::move(halt_condition);
  }

  for (std::size_t i = 0; i < num_joints_; ++i) {
    float cfg_red = mechanical_reductions_[i].load();
    std::uint32_t enc_res = encoder_resolutions_[i].load();
    std::int32_t si_vel = si_velocity_units_[i].load();
    auto converted = OutputShaftRadPerSToVelocityValue(velocities[i], si_vel,
                                                       cfg_red, enc_res);
    const auto converted_i32 = static_cast<std::int32_t>(converted);
    spdlog::debug(
        "SetVelocityCommand joint {}: input_rad_s={:.6f}, si_velocity_unit=0x{:08X} ({}), "
        "mech_red={:.6f}, enc_res={}, converted_value={}, PDO TargetVelocity(int32)={}",
        i, velocities[i], static_cast<std::uint32_t>(si_vel), si_vel, cfg_red,
        enc_res, converted, converted_i32);
    threadsafe_commands_velocities_[i] = static_cast<float>(converted);
  }
  const int64_t now_ns =
      std::chrono::steady_clock::now().time_since_epoch().count();
  for (std::size_t i = 0; i < num_joints_; ++i) {
    last_velocity_write_time_ns_[i].store(now_ns, std::memory_order_release);
  }
  return {};
}

std::expected<void, Error> SynapticonBase::SetTorqueCommand(
    const std::vector<float>& torques,
    std::function<bool()> halt_condition) {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  if (torques.size() != num_joints_) {
    return std::unexpected(Error{ErrorCode::kInvalidArgument,
                                 "torques size mismatch"});
  }

  bool need_switch = false;
  for (std::size_t i = 0; i < num_joints_; ++i) {
    if (control_mode_[i] != ControlMode::kTorque) {
      need_switch = true;
      break;
    }
  }
  if (need_switch) {
    auto result = SwitchControlMode(ControlMode::kTorque);
    if (!result) return result;
  }

  {
    std::lock_guard<std::mutex> lock(halt_mutex_);
    halt_condition_ = std::move(halt_condition);
  }

  for (std::size_t i = 0; i < num_joints_; ++i) {
    threadsafe_commands_efforts_[i] = std::clamp(torques[i], -1000.0f, 1000.0f);
  }
  return {};
}

// ============================================================================
// Generic commands
// ============================================================================

std::expected<void, Error> SynapticonBase::SendCommand(
    const std::vector<float>& joint_commands) {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  if (joint_commands.size() != num_joints_) {
    return std::unexpected(Error{ErrorCode::kInvalidArgument,
                                 "commands size mismatch"});
  }

  for (std::size_t i = 0; i < num_joints_; ++i) {
    float mech_red = mechanical_reductions_[i].load();
    std::uint32_t enc_res = encoder_resolutions_[i].load();

    switch (control_mode_[i]) {
      case ControlMode::kPosition:
        threadsafe_commands_positions_[i] = static_cast<float>(
            OutputShaftRadToInputTicks(joint_commands[i],
                                       position_reductions_[i].load(), enc_res,
                                       startup_angle_wrap_value_[i]));
        break;
      case ControlMode::kVelocity:
        threadsafe_commands_velocities_[i] = static_cast<float>(
            OutputShaftRadPerSToVelocityValue(
                joint_commands[i], si_velocity_units_[i].load(), mech_red,
                enc_res));
        last_velocity_write_time_ns_[i].store(
            std::chrono::steady_clock::now().time_since_epoch().count(),
            std::memory_order_release);
        break;
      case ControlMode::kTorque:
        threadsafe_commands_efforts_[i] =
            std::clamp(joint_commands[i], -1000.0f, 1000.0f);
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

std::expected<void, Error> SynapticonBase::SetTrajectory(
    const std::vector<float>& /*positions*/,
    const std::vector<float>& /*time_from_start*/) {
  return std::unexpected(
      Error{ErrorCode::kInvalidMode, "Trajectory control not yet implemented"});
}

// ============================================================================
// State queries
// ============================================================================

std::expected<std::vector<float>, Error> SynapticonBase::GetPositions() const {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  std::lock_guard<std::mutex> lock(hw_state_mtx_);
  return state_positions_;
}

std::expected<std::vector<float>, Error> SynapticonBase::GetVelocities() const {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  std::lock_guard<std::mutex> lock(hw_state_mtx_);
  return state_velocities_;
}

std::expected<std::vector<float>, Error> SynapticonBase::GetTorques() const {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  std::lock_guard<std::mutex> lock(hw_state_mtx_);
  return state_torques_;
}

std::expected<std::vector<float>, Error> SynapticonBase::GetAccelerations() const {
  if (!control_loop_running_) {
    return std::unexpected(Error{ErrorCode::kControlLoopNotRunning,
                                 "Control loop not running"});
  }
  std::lock_guard<std::mutex> lock(hw_state_mtx_);
  return state_accelerations_;
}

std::expected<std::vector<JointLimitsInfo>, Error>
SynapticonBase::GetJointLimits() const {
  std::vector<JointLimitsInfo> limits(num_joints_);
  for (std::size_t i = 0; i < num_joints_; ++i) {
    limits[i].has_limits = has_position_limits_[i];
    limits[i].min_position = min_position_limits_[i];
    limits[i].max_position = max_position_limits_[i];
  }
  return limits;
}

// ============================================================================
// Internal helpers
// ============================================================================

void SynapticonBase::UpdateInSomanetSnapshotLocked() {
  for (std::size_t joint_idx = 0; joint_idx < num_joints_; ++joint_idx) {
    if (in_somanet_[joint_idx] == nullptr) continue;
    in_somanet_snapshot_[joint_idx].position_value =
        in_somanet_[joint_idx]->PositionValue;
    in_somanet_snapshot_[joint_idx].velocity_value =
        in_somanet_[joint_idx]->VelocityValue;
    in_somanet_snapshot_[joint_idx].torque_value =
        in_somanet_[joint_idx]->TorqueValue;
  }
}

void SynapticonBase::WaitForGoodProcessData() {
  int wkc = -1;
  ec_send_processdata();
  wkc = ec_receive_processdata(EC_TIMEOUTRET);
  while (wkc < 0) {
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    osal_usleep(100000);
  }
}

void SynapticonBase::HandleStartupFaults() {
  ec_readstate();
  for (int slave = 1; slave <= ec_slavecount; slave++) {
    if (std::strcmp(ec_slave[slave].name, kExpectedSlaveName.data()) != 0) continue;
    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
      spdlog::warn("Slave {} is in SAFE_OP + ERROR, attempting ack", slave);
      ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
      ec_writestate(slave);
      osal_usleep(100000);
    }
  }
}

bool SynapticonBase::IsEStopEngaged() {
  ec_send_processdata();
  wkc_ = ec_receive_processdata(EC_TIMEOUTRET);

  if (wkc_ < expected_wkc_) {
    if (pdo_exchange_count_.load() < kMinPdoExchanges) {
      return true;
    }
    static auto last_process_data_warn =
        std::chrono::steady_clock::time_point{};
    const auto now = std::chrono::steady_clock::now();
    if (now - last_process_data_warn >= kProcessDataWarnThrottleInterval) {
      spdlog::warn("Process data communication failed");
      last_process_data_warn = now;
    }
    return true;
  }
  return false;
}

void SynapticonBase::StatusWordErrorLogging(std::size_t joint_idx) {
  if (in_somanet_[joint_idx]->Statusword & (1 << 11)) {
    spdlog::warn(
        "Internal limit active for joint {}, TorqueDemand: {}, "
        "VelocityDemandValue: {}",
        joint_idx, in_somanet_[joint_idx]->TorqueDemand,
        in_somanet_[joint_idx]->VelocityDemandValue);
  }
}

// ============================================================================
// EtherCAT error monitoring
// ============================================================================

void SynapticonBase::EcatCheck(std::stop_token stop_token) {
  std::uint8_t currentgroup = 0;

  while (!stop_token.stop_requested() &&
         !shutdown_requested_.load(std::memory_order_acquire)) {
    {
      std::lock_guard<std::mutex> lock(ecat_mtx_);
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
    }
    osal_usleep(10000);
  }
}

// ============================================================================
// Main control loop
// ============================================================================

void SynapticonBase::ControlLoop(std::stop_token stop_token) {
  std::vector<bool> first_iteration(num_joints_, true);

  while (!stop_token.stop_requested() &&
         !shutdown_requested_.load(std::memory_order_acquire)) {
    bool estop = false;
    bool halt_triggered = false;
    {
      std::scoped_lock lock(ecat_mtx_, hw_state_mtx_);
      estop = IsEStopEngaged();
      if (wkc_.load() >= expected_wkc_.load()) {
        UpdateInSomanetSnapshotLocked();
        if (pdo_exchange_count_.load() < kMinPdoExchanges) {
          pdo_exchange_count_.fetch_add(1);
        }
      }
      if (estop) {
        require_new_command_mode_ = true;
      }

      if (pdo_exchange_count_.load() >= kMinPdoExchanges) {
        static auto last_debug_log = std::chrono::steady_clock::time_point{};
        const auto now_dbg = std::chrono::steady_clock::now();
        const bool should_log_debug =
            (now_dbg - last_debug_log) >= std::chrono::milliseconds(500);
        if (should_log_debug) last_debug_log = now_dbg;

        for (std::size_t joint_idx = 0; joint_idx < num_joints_; ++joint_idx) {
          const auto& snapshot = in_somanet_snapshot_[joint_idx];
          state_velocities_[joint_idx] = VelocityValueToOutputShaftRadPerS(
              snapshot.velocity_value, si_velocity_units_[joint_idx].load(),
              mechanical_reductions_[joint_idx].load(),
              encoder_resolutions_[joint_idx].load());

          std::call_once(startup_angle_wrap_flag_[joint_idx], [&]() {
            ComputeStartupWrapOffset(
                snapshot.position_value,
                position_reductions_[joint_idx].load(),
                encoder_resolutions_[joint_idx].load(),
                startup_angle_wrap_value_[joint_idx]);
          });

          state_positions_[joint_idx] = InputTicksToOutputShaftRad(
              snapshot.position_value, position_reductions_[joint_idx].load(),
              encoder_resolutions_[joint_idx].load(),
              startup_angle_wrap_value_[joint_idx]);
          state_torques_[joint_idx] =
              (snapshot.torque_value / 1000.0f) *
              rated_torques_[joint_idx].load() *
              mechanical_reductions_[joint_idx].load();
          state_accelerations_[joint_idx] = 0.0f;

          if (should_log_debug) {
            spdlog::debug(
                "ControlLoop joint {} state: raw_pos_ticks={}, raw_vel={}, "
                "pos_reduction={:.6f}, enc_res={}, wrap_value={:.6f}, "
                "computed_pos_rad={:.6f}, computed_vel_rad_s={:.6f}, "
                "si_vel_unit=0x{:08X}, mech_red={:.6f}",
                joint_idx, snapshot.position_value, snapshot.velocity_value,
                position_reductions_[joint_idx].load(),
                encoder_resolutions_[joint_idx].load(),
                startup_angle_wrap_value_[joint_idx].load(std::memory_order_relaxed),
                state_positions_[joint_idx], state_velocities_[joint_idx],
                static_cast<std::uint32_t>(si_velocity_units_[joint_idx].load()),
                mechanical_reductions_[joint_idx].load());
          }
        }
      }
    }

    if (estop) {
      osal_usleep(kCyclicLoopSleepUs);
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(halt_mutex_);
      if (halt_condition_ && halt_condition_()) {
        spdlog::info("Halt condition triggered, stopping all joints");
        {
          std::scoped_lock hw_lock(ecat_mtx_, hw_state_mtx_);
          for (std::size_t i = 0; i < num_joints_; ++i) {
            Stop(out_somanet_[i], true);
          }
        }
        halt_condition_ = nullptr;
        halt_triggered = true;
      }
    }

    if (halt_triggered) {
      osal_usleep(kCyclicLoopSleepUs);
      continue;
    }

    {
      std::scoped_lock lock(ecat_mtx_, hw_state_mtx_);

      if (require_new_command_mode_) {
        allow_mode_change_ = true;
      } else {
        OnPreStateMachine();

        const bool mode_switch = mode_switch_in_progress_;

        for (std::size_t joint_idx = 0; joint_idx < num_joints_; ++joint_idx) {
          if (first_iteration[joint_idx]) {
            out_somanet_[joint_idx]->OpMode = kProfileTorqueMode;
            first_iteration[joint_idx] = false;
          }

          BaseStateMachineStep(joint_idx, mode_switch);
          StatusWordErrorLogging(joint_idx);
        }
      }
    }

    OnPostCycle();
    osal_usleep(kCyclicLoopSleepUs);
  }
}

// ============================================================================
// Per-joint state machine
// ============================================================================

void SynapticonBase::BaseStateMachineStep(std::size_t joint_idx,
                                          bool mode_switch) {
  const auto statusword = in_somanet_[joint_idx]->Statusword;
  const bool hold_in_sd = hold_in_shutdown_[joint_idx].load();

  // Fault
  if ((statusword & 0x004F) == 0x0008) {
    HandleFault(in_somanet_[joint_idx], out_somanet_[joint_idx], joint_idx);
    return;
  }
  // Switch on disabled
  if ((statusword & 0x004F) == 0x0040) {
    HandleShutdown(out_somanet_[joint_idx], control_mode_[joint_idx],
                   mode_switch_in_progress_, hold_in_sd);
    return;
  }
  // Ready to switch on
  if ((statusword & 0x006F) == 0x0021) {
    HandleSwitchOn(out_somanet_[joint_idx], hold_in_sd);
    return;
  }
  // Switched on
  if ((statusword & 0x006F) == 0x0023) {
    HandleEnableOperation(out_somanet_[joint_idx], mode_switch_in_progress_,
                          hold_in_sd);
    return;
  }
  // Quick stop active
  if ((statusword & 0x006F) == 0x0007) {
    if (control_mode_[joint_idx] != ControlMode::kQuickStop) {
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

  // Give derived class first chance to handle the joint
  if (OnNormalOperation(joint_idx, mode_switch)) {
    return;
  }

  // --- Base mode dispatch ---
  if (control_mode_[joint_idx] == ControlMode::kVelocity) {
    if (!std::isnan(threadsafe_commands_velocities_[joint_idx])) {
      const int64_t now_ns =
          std::chrono::steady_clock::now().time_since_epoch().count();
      const int64_t last_ns =
          last_velocity_write_time_ns_[joint_idx].load(std::memory_order_acquire);
      const bool timed_out =
          (last_ns > 0) &&
          (now_ns - last_ns > VELOCITY_COMMAND_TIMEOUT_NS);

      if (timed_out) {
        out_somanet_[joint_idx]->TargetVelocity = 0;
        out_somanet_[joint_idx]->OpMode = kCyclicVelocityMode;
        out_somanet_[joint_idx]->VelocityOffset = 0;
        if (!mode_switch) {
          out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
        }
        spdlog::debug(
            "BaseStateMachineStep joint {} VELOCITY TIMED OUT: "
            "TargetVelocity=0, OpMode={}, Controlword=0x{:04X}, "
            "feedback VelocityValue={}, feedback OpModeDisplay={}",
            joint_idx, out_somanet_[joint_idx]->OpMode,
            out_somanet_[joint_idx]->Controlword,
            in_somanet_[joint_idx]->VelocityValue,
            in_somanet_[joint_idx]->OpModeDisplay);
      } else {
        out_somanet_[joint_idx]->TargetVelocity = static_cast<std::int32_t>(
            threadsafe_commands_velocities_[joint_idx].load());
        out_somanet_[joint_idx]->OpMode = kCyclicVelocityMode;
        out_somanet_[joint_idx]->VelocityOffset = 0;
        if (!mode_switch) {
          out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
        }
        spdlog::debug(
            "BaseStateMachineStep joint {} VELOCITY: "
            "TargetVelocity={}, cmd_buf_f={:.1f}, OpMode={}, Controlword=0x{:04X}, "
            "mode_switch={}, feedback VelocityValue={}, feedback OpModeDisplay={}",
            joint_idx, out_somanet_[joint_idx]->TargetVelocity,
            threadsafe_commands_velocities_[joint_idx].load(),
            out_somanet_[joint_idx]->OpMode,
            out_somanet_[joint_idx]->Controlword,
            mode_switch,
            in_somanet_[joint_idx]->VelocityValue,
            in_somanet_[joint_idx]->OpModeDisplay);
      }
    }
  } else if (control_mode_[joint_idx] == ControlMode::kPosition) {
    if (!std::isnan(threadsafe_commands_positions_[joint_idx])) {
      out_somanet_[joint_idx]->TargetPosition =
          static_cast<std::int32_t>(threadsafe_commands_positions_[joint_idx].load());
      out_somanet_[joint_idx]->OpMode = kCyclicPositionMode;
      out_somanet_[joint_idx]->VelocityOffset = 0;
      if (!mode_switch) {
        out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
      }
    }
  } else if (control_mode_[joint_idx] == ControlMode::kQuickStop) {
    Stop(out_somanet_[joint_idx], true);
  } else if (control_mode_[joint_idx] == ControlMode::kTorque) {
    if (!std::isnan(threadsafe_commands_efforts_[joint_idx])) {
      out_somanet_[joint_idx]->TargetTorque =
          static_cast<std::int16_t>(threadsafe_commands_efforts_[joint_idx].load());
      out_somanet_[joint_idx]->OpMode = kProfileTorqueMode;
      if (!mode_switch) {
        out_somanet_[joint_idx]->Controlword = kNormalOpBrakesOff;
      }
    }
  } else if (control_mode_[joint_idx] == ControlMode::kUndefined) {
    out_somanet_[joint_idx]->OpMode = kProfileTorqueMode;
    out_somanet_[joint_idx]->TorqueOffset = 0;
    out_somanet_[joint_idx]->Controlword = kQuickStopCtrlWord;
  }
}

}  // namespace elevated_control
