/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @file virtual_bumper.cpp
/// @brief Virtual bumper base class
#include "virtual_bumper.hpp"
#include "param.hpp"

namespace {
// ROS parameter name
const char* kMaxScaleVelocity = "max_scale_velocity";  // Velocity at which it becomes 100% size [m/s]
const char* kMinScaleVelocity = "min_scale_velocity";  // Velocity at which it becomes the minimum size [m/s]
const char* kMinScale = "min_scale";                   // Minimum size scale [-]
const char* kAutoScaling = "auto_scaling";             // Bumper auto-correction definition enumeration
// ROS parameter default values
const double kMaxScaleVelocityDef = 0.4;  // Default value for velocity at which it becomes 100% size [m/s]
const double kMinScaleVelocityDef = 0.2;  // Default value for velocity at which it becomes the minimum size [m/s]
const double kMinScaleDef = 1.0;          // Default value for minimum size scale [-]
}  // anonymous namespace

namespace tmc_safety_velocity_limiter {
/// Constructor
VirtualBumper::VirtualBumper(
    std::map<std::string, rclcpp::Parameter>& parameters, const VelocitySlope::Ptr& velocity_slope) :
    velocity_slope_(velocity_slope) {
  // Retrieve common parameters
  GetCommonParameters(parameters);
}

/// Determine the bumper size scale from the magnitude of the moving velocity
/// @param input_velocity [I] Moving velocity
/// @return Bumper size scale
double VirtualBumper::CalcBumperScale(const Twist& input_velocity) {
  if (min_scale_ >= 1.0) {
    // Setting without auto_scaling. Returns unity
    return 1.0;
  }
  const double composite_velocity = sqrt(pow(input_velocity.linear.x, 2.0) +
                                         pow(input_velocity.linear.y, 2.0));
  if (composite_velocity < min_scale_velocity_) {
    // Returns the minimum scale if below min_scale_velocity_
    return min_scale_;
  } else if (composite_velocity < max_scale_velocity_) {
    // Returns a scale according to the moving velocity if between parameters
    const double bumper_scale = min_scale_ + (1.0 - min_scale_) *
        ((composite_velocity - min_scale_velocity_) / (max_scale_velocity_ - min_scale_velocity_));
    return bumper_scale;
  } else {
    // Returns unity if above max_scale_velocity_
    return 1.0;
  }
}

/// Retrieve common parameters not affected by derived classes
void VirtualBumper::GetCommonParameters(std::map<std::string, rclcpp::Parameter>& parameters) {
  // Read parameters below auto_scaling
  std::map<std::string, rclcpp::Parameter> auto_scaling_param;
  if (GetGroupParam(parameters, kAutoScaling, auto_scaling_param)) {
    GetOptionalParam(auto_scaling_param, kMaxScaleVelocity, max_scale_velocity_, kMaxScaleVelocityDef);
    GetOptionalParam(auto_scaling_param, kMinScaleVelocity, min_scale_velocity_, kMinScaleVelocityDef);
    if (max_scale_velocity_ < 0.0 || min_scale_velocity_ < 0.0 || max_scale_velocity_ <= min_scale_velocity_) {
      RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
          "Parameter [%s] and [%s] is invalid %lf, %lf. Use default value %lf, %lf",
          kMaxScaleVelocity, kMinScaleVelocity, max_scale_velocity_, min_scale_velocity_,
          kMaxScaleVelocityDef, kMinScaleVelocityDef);
      max_scale_velocity_ = kMaxScaleVelocityDef;
      min_scale_velocity_ = kMinScaleVelocityDef;
    }
    GetOptionalParam(auto_scaling_param, kMinScale, min_scale_, kMinScaleDef);
    if (min_scale_ < 0.0 || min_scale_ > 1.0) {
      RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
          "Parameter [%s] is invalid %lf. Use default value %lf",
          kMinScale, min_scale_, kMinScaleDef);
      min_scale_ = kMinScaleDef;
    }
  } else {
    // If auto_scaling is not set, the bumper size remains constant
    min_scale_ = 1.0;
    max_scale_velocity_ = kMaxScaleVelocityDef;
    min_scale_velocity_ = kMinScaleVelocityDef;
  }
}
}  // namespace tmc_safety_velocity_limiter
