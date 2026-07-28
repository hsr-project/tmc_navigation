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
#include <tmc_base_path_follower/velocity_calculator_omni.hpp>

#include <algorithm>

#include <angles/angles.h>

namespace tmc_base_path_follower {

bool OmniVelocityCalculator::CalculateVelocity(
    const PathInfo& path_info, const Pose2d& global_pose, const uint32_t current_path_index,
    const Vector3d& last_velocity, const double time_interval, const bool is_arrived_goal_area,
    const std::optional<double>& transit_velocity, Vector3d& output_velocity) {
  // Control is divided between precise approach to the goal point near the goal area and path-following control elsewhere.
  if (is_arrived_goal_area) {
    // Calculation of cart speed for approaching the goal.
    output_velocity = CalculateApproachGoalVelocity(path_info, global_pose);
  } else {
    // Stop when the nearest point is the last point even though it is not within the goal area.
    if (current_path_index == path_info.splined_path.size() - 1) {
      output_velocity = Vector3d::Zero();
      return false;
    }
    // Calculation of cart speed for following the path.
    output_velocity = CalculateFollowPathVelocity(path_info, global_pose, current_path_index, transit_velocity);
  }
  // Speed limit.
  const Vector3d max_velocity(param_.max_linear_velocity, param_.max_linear_velocity, param_.max_angular_velocity);
  const Vector3d max_acceleration(param_.max_linear_acceleration, param_.max_linear_acceleration,
      param_.max_angular_acceleration);
  LimitVelocity(output_velocity, last_velocity, time_interval, max_velocity, max_acceleration);

  return true;
}

/// Calculation of cart speed for following the path.
Vector3d OmniVelocityCalculator::CalculateFollowPathVelocity(
    const PathInfo& path_info, const Pose2d& global_pose, const uint32_t current_path_index,
    const std::optional<double>& transit_velocity) {
  Vector3d velocity = Vector3d::Zero();
  const Pose2d nearest_point = path_info.splined_path[current_path_index];
  const Pose2d goal = path_info.splined_path.back();
  // Calculation of translational speed.
  // Set translational speed proportional to the distance from the goal.
  double velocity_norm = (param_.velocity_margin +
      (goal.point() - global_pose.point()).norm() * param_.goal_deceleration);
  // Apply a limit based on the maximum speed restriction.
  velocity_norm = std::min<double>(velocity_norm, param_.max_linear_velocity);
  if (transit_velocity) {
    // Apply a limit based on the passing speed.
    velocity_norm = std::min<double>(velocity_norm, transit_velocity.value());
  }

  // Decompose into xy velocities.
  velocity(kPoseX) = velocity_norm * cos(nearest_point.theta());
  velocity(kPoseY) = velocity_norm * sin(nearest_point.theta());

  // Calculate feedback-based speed from the difference with the nearest point on the path.
  const double tangent = tan(nearest_point.theta());
  const double dx = nearest_point.x() - global_pose.x();
  const double dy = nearest_point.y() - global_pose.y();
  velocity(kPoseX) += param_.linear_p_gain * (tangent * tangent * dx - tangent * dy) / (tangent * tangent + 1.0);
  velocity(kPoseY) += param_.linear_p_gain * (dy - tangent * dx) / (tangent * tangent + 1.0);

  // Calculation of rotational speed.
  const double base_angular_velocity = velocity_norm * path_info.splined_path_curvatures[current_path_index];

  // Switch the orientation adjustment based on the length of the path to the goal.
  const double path_length = path_info.splined_path_left_lengths[current_path_index];
  if (path_length < param_.path_length_threshold) {
    const double dt_from_goal = angles::shortest_angular_distance(global_pose.theta(), goal.theta());
    velocity(kPoseTheta) = param_.goal_angle_gain * param_.max_linear_velocity * dt_from_goal / path_length;
  } else {
    const double dt_from_path = angles::shortest_angular_distance(global_pose.theta(), nearest_point.theta());
    velocity(kPoseTheta) = base_angular_velocity + param_.angular_p_gain * dt_from_path;
  }

  // Convert translational speed to the current cart orientation reference.
  Eigen::Matrix2d rotation;
  rotation << cos(global_pose.theta()), sin(global_pose.theta()), -sin(global_pose.theta()),
      cos(global_pose.theta());

  Vector3d output_velocity = Vector3d::Zero();
  output_velocity.head(2) = rotation * velocity.head(2);
  output_velocity(kPoseTheta) = velocity(kPoseTheta);

  return output_velocity;
}

/// Calculation of cart speed for approaching the goal.
Vector3d OmniVelocityCalculator::CalculateApproachGoalVelocity(
    const PathInfo& path_info, const Pose2d& global_pose) {
  Vector3d velocity = Vector3d::Zero();
  const Pose2d goal = path_info.splined_path.back();

  // Calculation of speed to move to the goal position.
  const Point2d linear_error = goal.point() - global_pose.point();
  const double angular_error = angles::shortest_angular_distance(global_pose.theta(), goal.theta());
  velocity(kPoseX) = param_.linear_p_gain * linear_error.x();
  velocity(kPoseY) = param_.linear_p_gain * linear_error.y();
  velocity(kPoseTheta) = param_.goal_angle_gain * angular_error;

  // Convert translational speed to the current cart orientation reference.
  Eigen::Matrix2d rotation;
  rotation << cos(global_pose.theta()), sin(global_pose.theta()), -sin(global_pose.theta()),
      cos(global_pose.theta());

  Vector3d output_velocity = Vector3d::Zero();
  output_velocity.head(2) = rotation * velocity.head(2);
  output_velocity(kPoseTheta) = velocity(kPoseTheta);

  return output_velocity;
}
}  // namespace tmc_base_path_follower
