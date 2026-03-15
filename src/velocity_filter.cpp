// Copyright (c) 2025 Elevate Robotics Inc

#include "elevated_control/velocity_filter.hpp"

namespace elevated_control {

VelocityFilter::VelocityFilter(float alpha) : alpha_(alpha) { Reset(); }

float VelocityFilter::Filter(float input) {
  float output = alpha_ * previous_output_ + (1.0f - alpha_) * input;
  previous_output_ = output;
  return output;
}

void VelocityFilter::Reset() { previous_output_ = 0.0f; }

}  // namespace elevated_control
