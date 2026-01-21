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
#include <tmc_base_path_follower/velocity_calculator_diff_drive.hpp>

#include <algorithm>
#include <limits>

#include <angles/angles.h>

namespace tmc_base_path_follower {

// If the angle deviates from the path, turn
// Once turning starts, continue turning until the direction aligns
// Follow the path
// If it exceeds the goal area, turn to align with the goal
bool DiffDriveVelocityCalculator::CalculateVelocity(
    const PathInfo& path_info, const Pose2d& global_pose, const uint32_t current_path_index,
    const Vector3d& last_velocity, const double time_interval, const bool is_arrived_goal_area,
    const std::optional<double>& transit_velocity, Vector3d& output_velocity) {
  if (is_arrived_goal_area) {
    // Cart speed calculation for turning towards the goal
    const double angular_error_to_goal = angles::shortest_angular_distance(
        global_pose.theta(), path_info.splined_path.back().theta());
    output_velocity = CalculateSpinVelocity(angular_error_to_goal);
  } else {
    // Stop if the nearest point is the last point but not within the goal area
    if (current_path_index == path_info.splined_path.size() - 1) {
      output_velocity = Vector3d::Zero();
      return false;
    }
    const double angular_error_to_path = angles::shortest_angular_distance(
        global_pose.theta(), path_info.splined_path[current_path_index].theta());

    // In-place turn determination
    // Start turning if the path and own direction are misaligned
    // If already turning, continue until the angle with the path is below a certain value
    if ((!is_spinning_to_path_angle_ && fabs(angular_error_to_path) > param_.spin_start_error_angle) ||
        (is_spinning_to_path_angle_ && fabs(angular_error_to_path) > param_.spin_end_error_angle)) {
      is_spinning_to_path_angle_ = true;
    } else {
      is_spinning_to_path_angle_ = false;
    }

    if (is_spinning_to_path_angle_) {
      // Cart speed calculation for turning in the path direction
      output_velocity = CalculateSpinVelocity(angular_error_to_path);
    } else {
      // Cart speed calculation for following the path
      output_velocity = CalculateForwardVelocity(path_info, global_pose, current_path_index,
          transit_velocity, angular_error_to_path);
    }
  }
  // Speed limit
  const Vector3d max_velocity(param_.max_linear_velocity, param_.max_linear_velocity, param_.max_angular_velocity);
  const Vector3d max_acceleration(param_.max_linear_acceleration, param_.max_linear_acceleration,
      param_.max_angular_acceleration);
  LimitVelocity(output_velocity, last_velocity, time_interval, max_velocity, max_acceleration);
  return true;
}

/// Travel speed calculation
Vector3d DiffDriveVelocityCalculator::CalculateForwardVelocity(
    const PathInfo& path_info, const Pose2d& global_pose, const uint32_t current_path_index,
    const std::optional<double>& transit_velocity, const double angular_error) {
  // Calculation of translational speed
  // Translational speed proportional to the distance from the goal
  double linear_velocity = (param_.velocity_margin +
      (path_info.splined_path.back().point() - global_pose.point()).norm() * param_.goal_deceleration);
  // Limit with maximum speed
  linear_velocity = std::min<double>(linear_velocity, param_.max_linear_velocity);
  if (transit_velocity) {
    // Limit with passing speed
    linear_velocity = std::min<double>(linear_velocity, transit_velocity.value());
  }
  Vector3d velocity = Vector3d::Zero();
  velocity(kPoseX) = linear_velocity;
  velocity(kPoseY) = 0.0;

  // Calculation of turning speed
  // FF calculation
  const double feedforward_velocity = linear_velocity * path_info.splined_path_curvatures[current_path_index];
  const Pose2d nearest_point = path_info.splined_path[current_path_index];
  // FB calculation
  // Calculate whether the self-position is deviated to the left or right relative to the path direction
  const Point2d diff = global_pose.point() - nearest_point.point();
  const double path_direction = nearest_point.theta();
  double linear_error_sign = diff.y() * cos(path_direction) - diff.x() * sin(path_direction);
  if (fabs(linear_error_sign) < std::numeric_limits<double>::epsilon()) {
    linear_error_sign = 0.0;
  } else {
    linear_error_sign = linear_error_sign / fabs(linear_error_sign);
  }
  const double linear_error = (nearest_point.point() - global_pose.point()).norm() * linear_error_sign;
  double feedback_velocity = (param_.linear_alpha_gain *
      (angular_error - param_.linear_beta_gain * linear_error) +
      param_.linear_beta_gain * fabs(linear_velocity) * sin(angular_error));
  velocity(kPoseTheta) = feedforward_velocity * cos(angular_error) + feedback_velocity;
  return velocity;
}

/// In-place turning speed calculation
Vector3d DiffDriveVelocityCalculator::CalculateSpinVelocity(const double angular_error) {
  Vector3d velocity = Vector3d::Zero();
  if (fabs(angular_error) > std::numeric_limits<double>::epsilon()) {
    const double sign_angular_error = angular_error / fabs(angular_error);
    // Calculate the magnitude of turning speed (minimum speed + speed according to angle error)
    double angular_velocity_abs = param_.spin_min_angular_velocity +
        fabs(angular_error) * param_.angle_error_angular_velocity_rate;
    // Do not exceed the maximum turning speed
    angular_velocity_abs = std::min<double>(angular_velocity_abs, param_.spin_max_angular_velocity);
    velocity(kPoseTheta) = angular_velocity_abs * sign_angular_error;
  }
  return velocity;
}
}  // namespace tmc_base_path_follower
