// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include <expected>

#include "elevated_control/interface_base.hpp"

namespace elevated_control {

// Convenience wrapper around SynapticonBase for single-motor testing.
// Scalar API; units match SynapticonBase: SetPosition/SetVelocity in rad and rad/s;
// SetTorque in per mille (‰) of rated torque; GetTorque in Nm at the output shaft.
class SingleJointInterface : public SynapticonBase {
 public:
  struct Config : SynapticonBaseConfig {
    Config() { expected_slave_count = 1; }
  };

  explicit SingleJointInterface(const Config& config);

  // Scalar convenience API (rad, rad/s, ‰ for set-torque; Nm for GetTorque).
  std::expected<void, Error> SetPosition(float position);
  std::expected<void, Error> SetVelocity(float velocity);
  std::expected<void, Error> SetTorque(float torque);

  std::expected<float, Error> GetPosition() const;
  std::expected<float, Error> GetVelocity() const;
  std::expected<float, Error> GetTorque() const;
};

}  // namespace elevated_control
