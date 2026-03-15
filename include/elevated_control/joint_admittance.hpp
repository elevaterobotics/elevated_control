// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

#include "elevated_control/velocity_filter.hpp"

namespace elevated_control {

// Single-joint admittance controller: M*q_dd + D*q_d = T_external
class JointAdmittance {
 public:
  bool Init(float M, float D);

  constexpr float inertia() const { return M_; }
  constexpr float damping() const { return D_; }

  float CalculateVelocity(float acceleration, float external_torque) {
    float velocity = (external_torque - M_ * acceleration) / D_;
    return velocity_filter_.Filter(velocity);
  }

  void Reset() { velocity_filter_.Reset(); }

 private:
  float M_ = 0.0f;
  float D_ = 0.0f;
  VelocityFilter velocity_filter_{0.98f};
};

}  // namespace elevated_control
