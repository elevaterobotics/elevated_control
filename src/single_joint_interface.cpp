// Copyright (c) 2025 Elevate Robotics Inc

#include "elevated_control/single_joint_interface.hpp"

namespace elevated_control {

SingleJointInterface::SingleJointInterface(const Config& config)
    : SynapticonBase(config) {}

std::expected<void, Error> SingleJointInterface::SetPosition(float position) {
  return SetPositionCommand({position});
}

std::expected<void, Error> SingleJointInterface::SetVelocity(float velocity) {
  return SetVelocityCommand({velocity});
}

std::expected<void, Error> SingleJointInterface::SetTorque(float torque) {
  return SetTorqueCommand({torque});
}

std::expected<float, Error> SingleJointInterface::GetPosition() const {
  auto result = GetPositions();
  if (!result) return std::unexpected(result.error());
  return result->at(0);
}

std::expected<float, Error> SingleJointInterface::GetVelocity() const {
  auto result = GetVelocities();
  if (!result) return std::unexpected(result.error());
  return result->at(0);
}

std::expected<float, Error> SingleJointInterface::GetTorque() const {
  auto result = GetTorques();
  if (!result) return std::unexpected(result.error());
  return result->at(0);
}

}  // namespace elevated_control
