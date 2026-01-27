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
#ifndef TMC_BASE_PATH_FOLLOWER_PATH_TRANSIT_VELOCITY_CALCULATOR_HPP_
#define TMC_BASE_PATH_FOLLOWER_PATH_TRANSIT_VELOCITY_CALCULATOR_HPP_
#include <memory>
#include <vector>

#include <console_bridge/console.h>
#include "common.hpp"
#include "parameter_default_value.hpp"

namespace tmc_base_path_follower {

class IPathTransitVelocityCalculator {
 public:
  using Ptr = std::shared_ptr<IPathTransitVelocityCalculator>;
  virtual ~IPathTransitVelocityCalculator() = default;
  virtual void CalculatePathTransitVelocity(const PathInfo& path_info) = 0;
  virtual double GetPathTransitVelocity(const uint32_t path_index) = 0;
};

/// Path Passing Speed Control Class
class PathTransitVelocityCalculator : public IPathTransitVelocityCalculator {
 public:
  /// Parameters
  struct Parameter {
    Parameter(const double in_max_linear_velocity,
              const double in_min_linear_velocity,
              const double in_max_angular_velocity,
              const double in_max_linear_acceleration,
              const double in_max_linear_deceleration,
              const double in_max_angular_acceleration,
              const double in_max_angular_deceleration,
              const double in_transit_velocity_angular_velocity_ratio)
        : max_linear_velocity(in_max_linear_velocity),
          min_linear_velocity(in_min_linear_velocity),
          max_angular_velocity(in_max_angular_velocity),
          max_linear_acceleration(in_max_linear_acceleration),
          max_linear_deceleration(in_max_linear_deceleration),
          max_angular_acceleration(in_max_angular_acceleration),
          max_angular_deceleration(in_max_angular_deceleration),
          transit_velocity_angular_velocity_ratio(in_transit_velocity_angular_velocity_ratio) {
      if (max_linear_velocity <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'max_linear_velocity' is invalid. Use default value.");
        max_linear_velocity = kMaxLinearVelocityDefault;
      }
      if (in_min_linear_velocity <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'min_linear_velocity' is invalid. Use default value.");
        min_linear_velocity = kMinLinearVelocityDefault;
      }
      if (min_linear_velocity > max_linear_velocity) {
        CONSOLE_BRIDGE_logWarn(
            "Value of 'max_linear_velocity' must greater than 'min_linear_velocity'. Use default value.");
        max_linear_velocity = kMaxLinearVelocityDefault;
        min_linear_velocity = kMinLinearVelocityDefault;
      }

      if (max_angular_velocity <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'max_angular_velocity' is invalid. Use default value.");
        max_angular_velocity = kMaxAngularVelocityDefault;
      }
      if (max_linear_acceleration <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'max_linear_acceleration' is invalid. Use default value.");
        max_linear_acceleration = kMaxLinearAccelerationDefault;
      }
      if (max_linear_deceleration <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'max_linear_deceleration' is invalid. Use default value.");
        max_linear_deceleration = kMaxLinearDecelerationDefault;
      }
      if (max_angular_acceleration <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'max_angular_acceleration' is invalid. Use default value.");
        max_angular_acceleration = kMaxAngularAccelerationDefault;
      }
      if (max_angular_deceleration <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'max_angular_deceleration' is invalid. Use default value.");
        max_angular_deceleration = kMaxAngularDecelerationDefault;
      }
      if (transit_velocity_angular_velocity_ratio <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'transit_velocity_angular_velocity_ratio' is invalid. Use default value.");
        transit_velocity_angular_velocity_ratio = kTransitVelocityAngularVelocityRatioDefault;
      }
    }
    // Maximum Translational Speed
    double max_linear_velocity;
    // Minimum Translational Speed
    double min_linear_velocity;
    // Maximum Rotational Speed
    double max_angular_velocity;
    // Maximum Translational Acceleration
    double max_linear_acceleration;
    // Maximum Translational Deceleration
    double max_linear_deceleration;
    // Maximum Rotational Acceleration
    double max_angular_acceleration;
    // Maximum Translational Deceleration
    double max_angular_deceleration;
    // Multiplier for Rotational Speed in Curvature-based Passing Speed Calculation
    double transit_velocity_angular_velocity_ratio;
  };

  /// Constructor
  /// @param [I] param Parameters
  explicit PathTransitVelocityCalculator(const Parameter& param) : param_(param) {}
  /// Path Passing Speed Calculation
  /// @param [I] path_info Path Information
  void CalculatePathTransitVelocity(const PathInfo& path_info);
  /// Path Passing Speed Control
  /// @param [I] path_index Index of Path Point
  /// @return Output Speed
  double GetPathTransitVelocity(const uint32_t path_index);

 private:
  // Parameters
  Parameter param_;
  // Passing Speed for Each Path Point
  std::vector<double> transit_velocity_;
};

}  // namespace tmc_base_path_follower

#endif  // TMC_BASE_PATH_FOLLOWER_PATH_TRANSIT_VELOCITY_CALCULATOR_HPP_
