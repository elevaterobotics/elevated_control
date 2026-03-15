// Copyright (c) 2025 Elevate Robotics Inc

#pragma once

namespace elevated_control {

// Low-pass filter for velocity or torque commands.
// Does not work for position because it resets to zero.
class VelocityFilter {
 public:
  explicit VelocityFilter(float alpha);
  float Filter(float input);
  void Reset();

 private:
  float alpha_;
  float previous_output_;
};

}  // namespace elevated_control
