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
/// @file     pose_integrator.cpp
/// @brief    Integrate multiple self-positioning results (library)
/// @version  0.2.0
/// @author   Takao Yasuda
/// @author   Applied for Partner-Robot Coding Rule(Ver:x.xx)
/// @date     2012.05.08
#include "pose_integrator.hpp"
#include <cmath>
#include <stdint.h>
#include <limits>
#include <string>
#include <rclcpp/rclcpp.hpp>

/// Anonymous namespace
namespace {
/// Convergence time for self-positioning
double const kConvergenceTime = 2.0;
/// Client operation cycle
double const kCycleTime = 2.0;
/// Reset time for external self-positioning estimation results
double const kPoseResetTime = 0.0;
}


/// Namespace (tmc_pose_integrator)
namespace tmc_pose_integrator {

/// Rotated posture for robot self-positioning
/// @param[in] in_pose Posture before rotation
/// @param[in] theta Rotation amount (rad)
/// @param[out] out_pose Posture before rotation
/// @return Posture after rotation. Orientation remains unchanged.
void RotatePoint(const Pose2d& in_pose, double theta, Pose2d& out_pose) {
  out_pose.x = cos(theta) * in_pose.x - sin(theta) * in_pose.y;
  out_pose.y = sin(theta) * in_pose.x + cos(theta) * in_pose.y;
}

/// Initialize each member variable and allocate data area
PoseIntegrator::PoseIntegrator()
    : is_first_odometry_received_(false),
      is_localization_updated_(false),
      convergence_time_(kConvergenceTime),
      cycle_time_(kCycleTime),
      time_from_pose_reset_(kPoseResetTime),
      previous_time_(0.0),
      stop_translational_vel_(0.0),
      stop_rotational_vel_(0.0),
      is_first_localization_(true) {
  current_adjusted_pose_.x = 0.0;
  current_adjusted_pose_.y = 0.0;
  current_adjusted_pose_.theta = 0.0;
  target_adjusted_pose_.x = 0.0;
  target_adjusted_pose_.y = 0.0;
  target_adjusted_pose_.theta = 0.0;
  adjusted_pose_at_localization_.x = 0.0;
  adjusted_pose_at_localization_.y = 0.0;
  adjusted_pose_at_localization_.theta = 0.0;
  corrected_odometry_.x = 0.0;
  corrected_odometry_.y = 0.0;
  corrected_odometry_.theta = 0.0;
  previous_odometry_.x = 0.0;
  previous_odometry_.y = 0.0;
  previous_odometry_.theta = 0.0;
}

/// Initialize members related to odometry only on the first occasion.
/// @param[in]  value Odometry data
void PoseIntegrator::set_odometry(const Pose2d& value) {
  odometry_ = value;

  // Reinitialize variables related to odometry only on the first occasion
  if (!is_first_odometry_received_) {
    // Save current odometry as reference odometry
    odometry_at_localization_update_ = odometry_;
    // Initialize variables for odometry difference calculation
    old_odometry_ = odometry_;
    // Set initialization completion flag
    is_first_odometry_received_ = true;
  }
}

/// @param[in]  value Odometry data synchronized with self-positioning and time
void PoseIntegrator::set_synchronized_odometry(const Pose2d& value) { synchronized_odometry_ = value; }

/// @param[in]  value 2D self-positioning estimation data
void PoseIntegrator::set_localized_2d_pose(const Pose2dWithCovariance& value) {
  localized_2d_pose_ = value;
  for (uint32_t i = 0; i < kCovarianceMatrixSize36; ++i) {
    localized_2d_pose_.covariance[i] = value.covariance[i];
  }
  is_localization_updated_ = true;
}

/// @param[in] value Convergence time (seconds)
void PoseIntegrator::set_convergence_time(double value) { convergence_time_ = value; }

/// @param[in] value Node operation cycle (seconds)
void PoseIntegrator::set_cycle_time(double value) { cycle_time_ = value; }

/// Determine whether the cart is moving based on odometry differences
bool PoseIntegrator::IsBaseMoving() {
  // Assume the cart is stationary on the first occasion
  if (is_first_localization_) {
    previous_odometry_ = odometry_;
    previous_time_ = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    is_first_localization_ = false;
    return false;
  }
  double diff_x = odometry_.x - previous_odometry_.x;
  double diff_y = odometry_.y - previous_odometry_.y;
  double diff_distance = sqrt(diff_x * diff_x + diff_y * diff_y);
  double diff_theta = fabs(odometry_.theta - previous_odometry_.theta);
  double delta_sec_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - previous_time_;
  previous_odometry_ = odometry_;
  previous_time_ = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
  if (diff_distance / delta_sec_time < stop_translational_vel_ && diff_theta / delta_sec_time < stop_rotational_vel_) {
    return false;
  }

  return true;
}

/// Generate corrected odometry using the difference between odometry synchronized with self-positioning and time and self-positioning
/// @return Self-positioning after convergence calculation
Pose2d PoseIntegrator::CorrectOdometryWithConvergenceAndSynchronization() {
  // Immediately return initial position if odometry has never been acquired
  if (!is_first_odometry_received_) {
    return corrected_odometry_;
  }
  Pose2d delay_pose, tmp_pose;
  // Enter this branch when laser self-positioning estimation is received.
  // Update correction parameters
  if (is_localization_updated_) {
    // Save reference odometry
    odometry_at_localization_update_ = odometry_;
    // Save reference corrected odometry.
    corrected_odometry_at_localization_ = corrected_odometry_;
    // Difference between synchronized odometry and current value: movement amount for time delay
    delay_pose.x = corrected_odometry_.x - synchronized_odometry_.x;
    delay_pose.y = corrected_odometry_.y - synchronized_odometry_.y;
    delay_pose.theta = corrected_odometry_.theta - synchronized_odometry_.theta;
    // Move laser_2d_pose for time delay
    RotatePoint(delay_pose, localized_2d_pose_.theta - synchronized_odometry_.theta, tmp_pose);
    localized_2d_pose_at_localization_update_.x = tmp_pose.x + localized_2d_pose_.x;
    localized_2d_pose_at_localization_update_.y = tmp_pose.y + localized_2d_pose_.y;
    localized_2d_pose_at_localization_update_.theta = localized_2d_pose_.theta + delay_pose.theta;

    // Reset timer to calculate correction coefficient
    time_from_pose_reset_ = 0.0;
    // Reset odometry movement amount
    diff_odometry_.ZeroClear();
    // Lower self-positioning update flag
    is_localization_updated_ = false;
  }

  // Using the set convergence time (convergence_time_) and,
  // elapsed time since the last self-positioning was set (time_from_pose_reset_),
  // calculate the current correction coefficient (0 to 1).
  double ratio = 0.0;
  if (!(convergence_time_ < std::numeric_limits<double>::epsilon())) {
    ratio = time_from_pose_reset_ / convergence_time_;
    if (ratio >= 1.0) {
      // If ratio exceeds 1 (= exceeds convergence time),
      // fix the correction coefficient to 1.
      ratio = 1.0;
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("pose_integrator"), "warning : convergence parameter must be more than zero");
    // If the convergence setting time is 0 or less, always set the correction coefficient to 1.
    ratio = 1.0;
  }

  // Calculate the difference (movement amount) of odometry
  diff_odometry_.x = diff_odometry_.x + (odometry_.x - old_odometry_.x);
  diff_odometry_.y = diff_odometry_.y + (odometry_.y - old_odometry_.y);
  diff_odometry_.theta = diff_odometry_.theta + (odometry_.theta - old_odometry_.theta);

  // Set orientation based on the origin
  Pose2d diff_odometry_from_localization;
  RotatePoint(diff_odometry_, -odometry_at_localization_update_.theta, diff_odometry_from_localization);

  // Convert angle to ±π
  diff_odometry_from_localization.theta =
      atan2(sin(diff_odometry_from_localization.theta), cos(diff_odometry_from_localization.theta));
  old_odometry_ = odometry_;

  // Calculate odometry movement amount relative to laser self-positioning orientation
  Pose2d diff_odometry_on_localized_pose_coordinate;
  RotatePoint(diff_odometry_from_localization, localized_2d_pose_at_localization_update_.theta,
              diff_odometry_on_localized_pose_coordinate);

  // Calculate the target corrected self-positioning
  Pose2d target_pose;
  target_pose.x = (localized_2d_pose_at_localization_update_.x + diff_odometry_on_localized_pose_coordinate.x);
  target_pose.y = (localized_2d_pose_at_localization_update_.y + diff_odometry_on_localized_pose_coordinate.y);

  // Use odometry movement amount as is for angle
  target_pose.theta = localized_2d_pose_at_localization_update_.theta + diff_odometry_.theta;
  target_pose.theta = atan2(sin(target_pose.theta), cos(target_pose.theta));

  // Calculate odometry movement amount relative to self-positioning when laser self-positioning is received
  Pose2d diff_odometry_on_corrected_odometry_coordinate;
  RotatePoint(diff_odometry_from_localization, corrected_odometry_at_localization_.theta,
              diff_odometry_on_corrected_odometry_coordinate);

  // Calculate self-positioning when laser self-positioning is not received
  Pose2d dying_pose;
  dying_pose.x = (corrected_odometry_at_localization_.x + diff_odometry_on_corrected_odometry_coordinate.x);
  dying_pose.y = (corrected_odometry_at_localization_.y + diff_odometry_on_corrected_odometry_coordinate.y);
  dying_pose.theta = (corrected_odometry_at_localization_.theta + diff_odometry_.theta);
  dying_pose.theta = atan2(sin(dying_pose.theta), cos(dying_pose.theta));

  // Add odometry movement amount and correction amount to reference corrected odometry.
  corrected_odometry_.x = ratio * target_pose.x + (1.0 - ratio) * dying_pose.x;
  corrected_odometry_.y = ratio * target_pose.y + (1.0 - ratio) * dying_pose.y;
  if (target_pose.theta - dying_pose.theta > M_PI) {
    dying_pose.theta = dying_pose.theta + (2.0 * M_PI);
  } else if ((target_pose.theta - dying_pose.theta) < -M_PI) {
    dying_pose.theta = dying_pose.theta - (2.0 * M_PI);
  }
  corrected_odometry_.theta = ratio * target_pose.theta + (1.0 - ratio) * dying_pose.theta;

  // Ideally, the measured time should be added to the timer
  time_from_pose_reset_ += cycle_time_;

  // Convert to ±π
  corrected_odometry_.theta = atan2(sin(corrected_odometry_.theta), cos(corrected_odometry_.theta));


  return corrected_odometry_;
}


/// Calculate corrected odometry using linear convergence
/// @return Self-positioning after convergence calculation
/// @todo Complement and acquire reference corrected odometry synchronized with laser self-positioning estimation and time
Pose2d PoseIntegrator::CorrectOdometryWithConvergence() {
  // Immediately return initial position if odometry has never been acquired
  if (!is_first_odometry_received_) {
    return corrected_odometry_;
  }

  // Enter this branch when laser self-positioning estimation is received.
  // Update correction parameters
  if (is_localization_updated_) {
    // Save reference odometry
    odometry_at_localization_update_ = odometry_;
    // Save reference corrected odometry.
    corrected_odometry_at_localization_ = corrected_odometry_;
    // Reset timer to calculate correction coefficient
    time_from_pose_reset_ = 0.0;
    // Reset odometry movement amount
    diff_odometry_.ZeroClear();
    // Lower self-positioning update flag
    is_localization_updated_ = false;
  }

  // Using the set convergence time (convergence_time_) and,
  // elapsed time since the last self-positioning was set (time_from_pose_reset_),
  // calculate the current correction coefficient (0 to 1).
  double ratio = 0.0;
  if (!(convergence_time_ < std::numeric_limits<double>::epsilon())) {
    ratio = time_from_pose_reset_ / convergence_time_;
    if (ratio >= 1.0) {
      // If ratio exceeds 1 (= exceeds convergence time),
      // fix the correction coefficient to 1.
      ratio = 1.0;
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("pose_integrator"), "warning : convergence parameter must be more than zero");
    // If the convergence setting time is 0 or less, always set the correction coefficient to 1.
    ratio = 1.0;
  }

  // Calculate the difference (movement amount) of odometry
  diff_odometry_.x = diff_odometry_.x + (odometry_.x - old_odometry_.x);
  diff_odometry_.y = diff_odometry_.y + (odometry_.y - old_odometry_.y);
  diff_odometry_.theta = diff_odometry_.theta + (odometry_.theta - old_odometry_.theta);

  // Set orientation based on the origin
  Pose2d diff_odometry_from_localization;
  RotatePoint(diff_odometry_, -odometry_at_localization_update_.theta, diff_odometry_from_localization);

  // Convert angle to ±π
  diff_odometry_from_localization.theta =
      atan2(sin(diff_odometry_from_localization.theta), cos(diff_odometry_from_localization.theta));
  old_odometry_ = odometry_;

  // Calculate odometry movement amount relative to laser self-positioning orientation
  Pose2d diff_odometry_on_localized_pose_coordinate;
  RotatePoint(diff_odometry_from_localization, localized_2d_pose_.theta, diff_odometry_on_localized_pose_coordinate);

  // Calculate the target corrected self-positioning
  Pose2d target_pose;
  target_pose.x = (localized_2d_pose_.x + diff_odometry_on_localized_pose_coordinate.x);
  target_pose.y = (localized_2d_pose_.y + diff_odometry_on_localized_pose_coordinate.y);

  // Use odometry movement amount as is for angle
  target_pose.theta = localized_2d_pose_.theta + diff_odometry_.theta;
  target_pose.theta = atan2(sin(target_pose.theta), cos(target_pose.theta));

  // Calculate odometry movement amount relative to self-positioning when laser self-positioning is received
  Pose2d diff_odometry_on_corrected_odometry_coordinate;
  RotatePoint(diff_odometry_from_localization, corrected_odometry_at_localization_.theta,
              diff_odometry_on_corrected_odometry_coordinate);

  // Calculate self-positioning when laser self-positioning is not received
  Pose2d dying_pose;
  dying_pose.x = (corrected_odometry_at_localization_.x + diff_odometry_on_corrected_odometry_coordinate.x);
  dying_pose.y = (corrected_odometry_at_localization_.y + diff_odometry_on_corrected_odometry_coordinate.y);
  dying_pose.theta = (corrected_odometry_at_localization_.theta + diff_odometry_.theta);
  dying_pose.theta = atan2(sin(dying_pose.theta), cos(dying_pose.theta));

  // Add odometry movement amount and correction amount to reference corrected odometry.
  corrected_odometry_.x = ratio * target_pose.x + (1.0 - ratio) * dying_pose.x;
  corrected_odometry_.y = ratio * target_pose.y + (1.0 - ratio) * dying_pose.y;
  if (target_pose.theta - dying_pose.theta > M_PI) {
    dying_pose.theta = dying_pose.theta + (2.0 * M_PI);
  } else if ((target_pose.theta - dying_pose.theta) < -M_PI) {
    dying_pose.theta = dying_pose.theta - (2.0 * M_PI);
  }
  corrected_odometry_.theta = ratio * target_pose.theta + (1.0 - ratio) * dying_pose.theta;

  // Ideally, the measured time should be added to the timer
  time_from_pose_reset_ += cycle_time_;

  // Convert to ±π
  corrected_odometry_.theta = atan2(sin(corrected_odometry_.theta), cos(corrected_odometry_.theta));


  return corrected_odometry_;
}

}  // namespace tmc_pose_integrator
