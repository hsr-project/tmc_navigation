/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
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
/// @file logarithm_slope.cpp
/// @brief Logarithmic velocity gradient
#include "logarithm_slope.hpp"
#include <boost/utility.hpp>

#include "param.hpp"

namespace {
// ROS parameter name
const char* kUpperLimitThreshold = "upper_limit_threshold";  // Upper limit of input ratio [-]
const char* kLowerLimitThreshold = "lower_limit_threshold";  // Lower limit of input ratio [-]
const char* kUpperLimitRatio = "upper_limit_ratio";          // Maximum output ratio [-]
const char* kLowerLimitRatio = "lower_limit_ratio";          // Linear lower output ratio [-]
const char* kMinimumRatio = "minimum_ratio";                 // Minimum output ratio [-]
const char* kLogarithmBase = "logarithm_base";               // Base of logarithm [-]

// ROS parameter default values
const double kUpperLimitThresholdDef = 1.0;
const double kLowerLimitThresholdDef = 0.0;
const double kUpperLimitRatioDef = 1.0;
const double kLowerLimitRatioDef = 0.0;
const double kMinimumRatioDef = 0.0;
const double kLogarithmBaseDef = 10;

}  // anonymous namespace

namespace tmc_safety_velocity_limiter {
/// Constructor
LogarithmSlope::LogarithmSlope(std::map<std::string, rclcpp::Parameter>& parameters) {
  UpdateParameters(parameters);
}

/// Calculate velocity ratio from input information
double LogarithmSlope::CalcRatio(const double distance_ratio) {
  if (distance_ratio > upper_limit_threshold_) {
    // Return max value if above the upper limit
    return upper_limit_ratio_;
  } else if (distance_ratio > lower_limit_threshold_) {
    // Scale input to range from 1.0 to logarithm_base,
    // and ensure log() output ranges from 0.0 to 1.0
    // between lower_limit_threshold and upper_limit_threshold
    double normalized_ratio = 1.0 +
                              (distance_ratio - lower_limit_threshold_) /
                              (upper_limit_threshold_ - lower_limit_threshold_) *
                              (logarithm_base_ - 1.0);

    double calculated_ratio = lower_limit_ratio_ +
                              (log(normalized_ratio) / log(logarithm_base_)) *
                              (upper_limit_ratio_ - lower_limit_ratio_);

    return calculated_ratio;
  } else {
    // Return min value if below the lower limit
    return minimum_ratio_;
  }
}

/// Retrieve ROS PARAM
void LogarithmSlope::UpdateParameters(std::map<std::string, rclcpp::Parameter>& parameters) {
  GetOptionalParam(parameters, kLowerLimitThreshold, lower_limit_threshold_, kLowerLimitThresholdDef);
  GetOptionalParam(parameters, kUpperLimitThreshold, upper_limit_threshold_, kUpperLimitThresholdDef);
  // Use default values if upper is not greater than lower or values are not in the range 0.0 to 1.0
  if (upper_limit_threshold_ <= lower_limit_threshold_ ||
      0.0 > upper_limit_threshold_ || 1.0 < upper_limit_threshold_ ||
      0.0 > lower_limit_threshold_ || 1.0 < lower_limit_threshold_) {
    RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
        "Parameter [%s-%s] is invalid [%lf-%lf]. Use default value [%lf-%lf]",
        kLowerLimitThreshold, kUpperLimitThreshold, lower_limit_threshold_, upper_limit_threshold_,
        kLowerLimitThresholdDef, kUpperLimitThresholdDef);
    lower_limit_threshold_ = kLowerLimitThresholdDef;
    upper_limit_threshold_ = kUpperLimitThresholdDef;
  }

  GetOptionalParam(parameters, kUpperLimitRatio, upper_limit_ratio_, kUpperLimitRatioDef);
  GetOptionalParam(parameters, kLowerLimitRatio, lower_limit_ratio_, kLowerLimitRatioDef);
  // Use default values if upper is not greater than lower or values are not in the range 0.0 to 1.0
  if (upper_limit_ratio_ <= lower_limit_ratio_ ||
      0.0 > upper_limit_ratio_ || 1.0 < upper_limit_ratio_ ||
      0.0 > lower_limit_ratio_ || 1.0 < lower_limit_ratio_) {
    RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
        "Parameter [%s-%s] is invalid [%lf-%lf]. Use default value [%lf-%lf]",
        kLowerLimitRatio, kUpperLimitRatio, lower_limit_ratio_, upper_limit_ratio_,
        kLowerLimitRatioDef, kUpperLimitRatioDef);
    lower_limit_ratio_ = kLowerLimitRatioDef;
    upper_limit_ratio_ = kUpperLimitRatioDef;
  }
  GetOptionalParam(parameters, kMinimumRatio, minimum_ratio_, kMinimumRatioDef);
  GetOptionalParam(parameters, kLogarithmBase, logarithm_base_, kLogarithmBaseDef);
}
}  // namespace tmc_safety_velocity_limiter
