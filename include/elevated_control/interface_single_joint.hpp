// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <expected>

#include "elevated_control/interface_base.hpp"

namespace elevated_control {

// Convenience wrapper around SynapticonBase for single-motor testing.
// Provides scalar (non-vector) API for position, velocity, and torque commands.
class SingleJointInterface : public SynapticonBase {
 public:
  struct Config : SynapticonBaseConfig {
    Config() { expected_slave_count = 1; }
  };

  explicit SingleJointInterface(const Config& config);

  // Scalar convenience API
  std::expected<void, Error> SetPosition(float position);
  std::expected<void, Error> SetVelocity(float velocity);
  std::expected<void, Error> SetTorque(float torque);

  std::expected<float, Error> GetPosition() const;
  std::expected<float, Error> GetVelocity() const;
  std::expected<float, Error> GetTorque() const;
};

}  // namespace elevated_control
