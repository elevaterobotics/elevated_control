// Copyright (c) 2025 Elevate Robotics Inc

#include "elevated_control/joint_admittance.hpp"

namespace elevated_control {
namespace {
inline constexpr float kEps = 1e-3f;
}

bool JointAdmittance::Init(const float M, const float D) {
  if (M <= kEps || D <= kEps) {
    return false;
  }
  M_ = M;
  D_ = D;
  return true;
}

}  // namespace elevated_control
