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
#ifndef TMC_BASE_PATH_FOLLOWER_VELOCITY_CALCULATOR_OMNI_HPP_
#define TMC_BASE_PATH_FOLLOWER_VELOCITY_CALCULATOR_OMNI_HPP_
#include <memory>

#include <console_bridge/console.h>
#include "common.hpp"
#include "parameter_default_value.hpp"
#include "velocity_calculator.hpp"

namespace tmc_base_path_follower {

/// Omni-directional model velocity calculation class
class OmniVelocityCalculator : public IVelocityCalculator {
 public:
  using Ptr = std::shared_ptr<OmniVelocityCalculator>;
  /// OmniVelocityCalculator parameters
  struct Parameter {
    Parameter(const double in_max_linear_velocity,
              const double in_max_angular_velocity,
              const double in_max_linear_acceleration,
              const double in_max_angular_acceleration,
              const double in_goal_deceleration,
              const double in_velocity_margin,
              const double in_path_length_threshold,
              const double in_linear_p_gain,
              const double in_angular_p_gain,
              const double in_goal_angle_gain)
        : max_linear_velocity(in_max_linear_velocity), max_angular_velocity(in_max_angular_velocity),
          max_linear_acceleration(in_max_linear_acceleration), max_angular_acceleration(in_max_angular_acceleration),
          goal_deceleration(in_goal_deceleration), velocity_margin(in_velocity_margin),
          path_length_threshold(in_path_length_threshold),
          linear_p_gain(in_linear_p_gain), angular_p_gain(in_angular_p_gain), goal_angle_gain(in_goal_angle_gain) {
      if (max_linear_velocity <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'max_linear_velocity' is invalid. Use default value.");
        max_linear_velocity = kMaxLinearVelocityDefault;
      }
      if (max_angular_velocity <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'max_angular_velocity' is invalid. Use default value.");
        max_angular_velocity = kMaxAngularVelocityDefault;
      }
      if (max_linear_acceleration <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'max_linear_acceleration' is invalid. Use default value.");
        max_linear_acceleration = kMaxLinearAccelerationDefault;
      }
      if (max_angular_acceleration <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'max_angular_acceleration' is invalid. Use default value.");
        max_angular_acceleration = kMaxAngularAccelerationDefault;
      }
      if (goal_deceleration <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'goal_deceleration' is invalid. Use default value.");
        goal_deceleration = kGoalDecelerationDefault;
      }
      if (velocity_margin <= 0.0 || velocity_margin > max_linear_velocity) {
        velocity_margin = kVelocityMarginDefault;
        CONSOLE_BRIDGE_logWarn("Value of 'velocity_margin' is invalid. Use default value.");
        if (max_linear_velocity <= velocity_margin) {
          // When the maximum translational velocity parameter is smaller than the velocity margin
          // Set default values for the maximum translational velocity parameter
          CONSOLE_BRIDGE_logWarn(
              "Value of 'max_linear_velocity' must greater than 'velocity_margin'. Use default value.");
          max_linear_velocity = kMaxLinearVelocityDefault;
        }
      }
      if (path_length_threshold <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'path_length_threshold' is invalid. Use default value.");
        path_length_threshold = kPathLengthThresholdDefault;
      }
      if (linear_p_gain <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'linear_p_gain' is invalid. Use default value.");
        linear_p_gain = kLinearPGainDefault;
      }
      if (angular_p_gain <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'angular_p_gain' is invalid. Use default value.");
        angular_p_gain = kAngularPGainDefault;
      }
      if (goal_angle_gain <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'goal_angle_gain' is invalid. Use default value.");
        goal_angle_gain = kGoalAngleGainDefault;
      }
    }
    // Maximum velocity
    double max_linear_velocity;
    double max_angular_velocity;

    // Maximum acceleration
    double max_linear_acceleration;
    double max_angular_acceleration;

    // Deceleration near the goal
    double goal_deceleration;
    // Minimum velocity near the goal
    double velocity_margin;
    // Distance to change the orientation of the upper body
    double path_length_threshold;

    // Feedback gain for position error
    double linear_p_gain;
    double angular_p_gain;
    double goal_angle_gain;
  };

  /// Constructor
  /// @param [I] param Parameters
  explicit OmniVelocityCalculator(const Parameter& param) : param_(param) {}


  /// Velocity calculation
  /// @param[I] path_info Path information
  /// @param[I] global_pose Self-position
  /// @param[I] current_path_index Index on the path
  /// @param[I] last_velocity Previous velocity
  /// @param[I] time_interval Time interval since the last velocity calculation
  /// @param[I] is_arrived_goal_area Whether it has entered the goal area
  /// @param[I] transit_velocity Transit velocity
  /// @param[O] output_velocity Output velocity
  /// @return Velocity calculation success or failure
  bool CalculateVelocity(const PathInfo& path_info, const Pose2d& global_pose, const uint32_t current_path_index,
                         const Vector3d& last_velocity, const double time_interval,
                         const bool is_arrived_goal_area, const std::optional<double>& transit_velocity,
                         Vector3d& output_velocity);

 private:
  /// Cart velocity calculation following the path
  Vector3d CalculateFollowPathVelocity(
      const PathInfo& path_info, const Pose2d& global_pose, const uint32_t current_path_index,
      const std::optional<double>& transit_velocity);
  /// Cart velocity calculation approaching the goal
  Vector3d CalculateApproachGoalVelocity(
      const PathInfo& path_info, const Pose2d& global_pose);

  // Parameters
  Parameter param_;
};
}  // namespace tmc_base_path_follower

#endif  // TMC_BASE_PATH_FOLLOWER_VELOCITY_CALCULATOR_OMNI_HPP_
