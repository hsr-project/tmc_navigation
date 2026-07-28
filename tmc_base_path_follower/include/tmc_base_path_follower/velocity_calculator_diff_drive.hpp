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
#ifndef TMC_BASE_PATH_FOLLOWER_VELOCITY_CALCULATOR_DIFF_DRIVE_HPP_
#define TMC_BASE_PATH_FOLLOWER_VELOCITY_CALCULATOR_DIFF_DRIVE_HPP_
#include <memory>

#include <console_bridge/console.h>

#include "common.hpp"
#include "parameter_default_value.hpp"
#include "velocity_calculator.hpp"

namespace tmc_base_path_follower {
/// Differential drive model velocity calculation class
class DiffDriveVelocityCalculator : public IVelocityCalculator {
 public:
  using Ptr = std::shared_ptr<DiffDriveVelocityCalculator>;
  /// DiffDriveVelocityCalculator parameters
  struct Parameter {
    Parameter(const double in_max_linear_velocity,
              const double in_max_angular_velocity,
              const double in_max_linear_acceleration,
              const double in_max_angular_acceleration,
              const double in_goal_deceleration,
              const double in_velocity_margin,
              const double in_linear_alpha_gain,
              const double in_linear_beta_gain,
              const double in_angle_error_angular_velocity_rate,
              const double in_spin_start_error_angle,
              const double in_spin_end_error_angle,
              const double in_spin_max_angular_velocity,
              const double in_spin_min_angular_velocity)
        : max_linear_velocity(in_max_linear_velocity), max_angular_velocity(in_max_angular_velocity),
          max_linear_acceleration(in_max_linear_acceleration), max_angular_acceleration(in_max_angular_acceleration),
          goal_deceleration(in_goal_deceleration), velocity_margin(in_velocity_margin),
          linear_alpha_gain(in_linear_alpha_gain), linear_beta_gain(in_linear_beta_gain),
          angle_error_angular_velocity_rate(in_angle_error_angular_velocity_rate),
          spin_start_error_angle(in_spin_start_error_angle), spin_end_error_angle(in_spin_end_error_angle),
          spin_max_angular_velocity(in_spin_max_angular_velocity),
          spin_min_angular_velocity(in_spin_min_angular_velocity) {
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
          // Set the default value for the maximum translational velocity parameter
          CONSOLE_BRIDGE_logWarn(
              "Value of 'max_linear_velocity' must greater than 'velocity_margin'. Use default value.");
          max_linear_velocity = kMaxLinearVelocityDefault;
        }
      }
      if (linear_alpha_gain < 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'linear_alpha_gain' is invalid. Use default value.");
        linear_alpha_gain = kLinearAlphaGainDefault;
      }
      if (linear_beta_gain < 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'linear_beta_gain' is invalid. Use default value.");
        linear_beta_gain = kLinearBetaGainDefault;
      }
      if (angle_error_angular_velocity_rate <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'angle_error_angular_velocity_rate' is invalid. Use default value.");
        angle_error_angular_velocity_rate = kAngleErrorAngularVelocityRateDefault;
      }

      if (spin_start_error_angle <= 0.0 || spin_end_error_angle <= 0.0 ||
          spin_start_error_angle < spin_end_error_angle) {
        CONSOLE_BRIDGE_logWarn(
            "Value of 'spin_start_error_angle' must greater than 'spin_end_error_angle'. Use default value.");
        spin_start_error_angle = kSpinStartErrorAngleDefault;
        spin_end_error_angle = kSpinEndErrorAngleDefault;
      }
      if (spin_max_angular_velocity <= 0.0 || spin_min_angular_velocity <= 0.0 ||
          spin_max_angular_velocity < spin_min_angular_velocity) {
        CONSOLE_BRIDGE_logWarn(
            "Value of 'spin_max_angular_velocity' must greater than 'spin_min_angular_velocity'. Use default value.");
        spin_max_angular_velocity = kSpinMaxAngularVelocityDefault;
        spin_min_angular_velocity = kSpinMinAngularVelocityDefault;
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
    // Distance to change body orientation
    double path_length_threshold;

    // Translational feedback gain
    double linear_alpha_gain;
    double linear_beta_gain;

    // Ratio of turning speed to angular error
    double angle_error_angular_velocity_rate;

    // Angular error to start in-place turning
    double spin_start_error_angle;
    // Angular error to end in-place turning
    double spin_end_error_angle;
    // Maximum turning speed for in-place turning
    double spin_max_angular_velocity;
    // Minimum turning speed for in-place turning
    double spin_min_angular_velocity;
  };
  /// Constructor
  /// @param [I] param Parameters
  explicit DiffDriveVelocityCalculator(const Parameter& param) : param_(param), is_spinning_to_path_angle_(false) {}


  /// Velocity calculation
  /// @param[I] path_info Path information
  /// @param[I] global_pose Self-position
  /// @param[I] current_path_index Index on the path
  /// @param[I] last_velocity Previous velocity
  /// @param[I] time_interval Time interval since the last velocity calculation
  /// @param[I] is_arrived_goal_area Whether it has entered the goal area
  /// @param[I] transit_velocity Transit velocity
  /// @param[O] output_velocity Output velocity
  /// @return Success or failure of velocity calculation
  bool CalculateVelocity(const PathInfo& path_info, const Pose2d& global_pose, const uint32_t current_path_index,
                         const Vector3d& last_velocity, const double time_interval,
                         const bool is_arrived_goal_area, const std::optional<double>& transit_velocity,
                         Vector3d& output_velocity);

 private:
  /// Progress velocity calculation
  Vector3d CalculateForwardVelocity(
      const PathInfo& path_info, const Pose2d& global_pose, const uint32_t current_path_index,
      const std::optional<double>& transit_velocity, const double angular_error);

  /// In-place turning velocity calculation
  Vector3d CalculateSpinVelocity(const double angular_error);

  // Parameters
  Parameter param_;
  // Whether it is turning toward the path direction
  bool is_spinning_to_path_angle_;
};

}  // namespace tmc_base_path_follower

#endif  // TMC_BASE_PATH_FOLLOWER_VELOCITY_CALCULATOR_DIFF_DRIVE_HPP_
