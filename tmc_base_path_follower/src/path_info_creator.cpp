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
#include <tmc_base_path_follower/path_info_creator.hpp>

#include <limits>
#include <vector>

#include <angles/angles.h>

namespace tmc_base_path_follower {

/// Route Information Generation
/// @param [I] path Input route
/// @return Route information
PathInfo PathInfoCreator::CreatePathInfo(const PoseSeq& path) {
  PathInfo path_info;
  path_info.origin_path = path;
  // Spline Interpolation Generation
  SplineInterpolation(path, path_info.splined_path, path_info.splined_path_curvatures);
  // Remaining Distance Generation for Spline Interpolated Route
  CalculateLeftPathLengths(path_info.splined_path, path_info.splined_path_left_lengths);

  return path_info;
}


/// @brief Spline Interpolation of Route
/// @param[in] input_path Pre-interpolation route
/// @param[out] splined_path Interpolated route
/// @param[out] splined_path_curvatures Curvature at each point of the interpolated route
void PathInfoCreator::SplineInterpolation(const PoseSeq& input_path, PoseSeq& splined_path,
    std::vector<double>& splined_path_curvatures) {
  // Perform interpolation for each point of the input route
  splined_path.clear();
  splined_path_curvatures.clear();
  PoseSeq path = input_path;
  // To not reflect the goal posture in the interpolation, insert a value calculated from the posture change from two points before
  if (path.size() > 2) {
    path.back().set_theta(angles::normalize_angle(path[path.size() - 2].theta() * 2 - path[path.size() - 3].theta()));
  }
  for (PoseSeq::const_iterator it = path.begin(); it != path.end() - 1; ++it) {
    PoseSeq::const_iterator next_it = it + 1;
    // Prepare for interpolation
    const double path_interval = ((*next_it).point() - (*it).point()).norm();
    const double dt = path_interval / param_.passing_velocity;
    /// Prevent division by zero. Skip if two points overlap
    /// In subsequent processing, dt, dt^2, dt^3 are used as divisors, but when dt is close to zero
    /// Check the smallest dt^3
    const double dt_cubic = dt * dt * dt;
    if (dt_cubic < std::numeric_limits<double>::epsilon()) {
      continue;
    }

    Eigen::Vector2d start_direction(cos((*it).theta()), sin((*it).theta()));
    Eigen::Vector2d end_direction(cos((*next_it).theta()), sin((*next_it).theta()));

    Eigen::Vector2d start_pos((*it).x(), (*it).y());
    Eigen::Vector2d start_vel = param_.passing_velocity * start_direction;
    Eigen::Vector2d end_pos((*next_it).x(), (*next_it).y());
    Eigen::Vector2d end_vel = param_.passing_velocity * end_direction;


    // Calculate coefficients for spline interpolation
    Eigen::Vector2d square_coeff = (-3.0 * start_pos + 3.0 * end_pos - 2.0 * dt * start_vel - dt * end_vel) / (dt * dt);
    Eigen::Vector2d cubic_coeff = (2.0 * start_pos - 2.0 * end_pos + dt * start_vel + dt * end_vel) / dt_cubic;

    Eigen::Vector4d coeff_x(start_pos(kPoseX), start_vel(kPoseX), square_coeff(kPoseX), cubic_coeff(kPoseX));
    Eigen::Vector4d coeff_y(start_pos(kPoseY), start_vel(kPoseY), square_coeff(kPoseY), cubic_coeff(kPoseY));

    // Check the extremum of the interpolation curve, and if there is a path that turns back, interpolate with a straight line
    if (CheckSplinePathExtremum(coeff_x, dt) && CheckSplinePathExtremum(coeff_y, dt)) {
      coeff_x << start_pos(kPoseX), (end_pos[kPoseX] - start_pos(kPoseX)) / dt, 0.0, 0.0;
      coeff_y << start_pos(kPoseY), (end_pos[kPoseY] - start_pos(kPoseY)) / dt, 0.0, 0.0;
    }

    // Curvature Calculation
    // Although accurate curvature can be calculated with spline interpolation, to avoid reflecting fine directional changes,
    // Use the rough curvature calculated from three adjacent points of the pre-interpolation input route
    double curvature = 0.0;
    if (path.size() > 2) {
      // Calculate curvature from the vector to the center point of the three points
      Point2d prev_to_curr;
      Point2d curr_to_next;
      if (it == path.begin()) {
        prev_to_curr = (it + 1)->point() - it->point();
        curr_to_next = (it + 2)->point() - (it + 1)->point();
      } else {
        prev_to_curr = it->point() - (it - 1)->point();
        curr_to_next = (it + 1)->point() - it->point();
      }
      const double cross_product = prev_to_curr.x() * curr_to_next.y() - prev_to_curr.y() * curr_to_next.x();
      const double dot_product = prev_to_curr.x() * curr_to_next.x() + prev_to_curr.y() * curr_to_next.y();
      const double diff_angle = atan2(cross_product, dot_product);
      if (curr_to_next.norm() > std::numeric_limits<double>::epsilon()) {
        curvature = diff_angle / curr_to_next.norm();
      }
    }

    // Perform interpolation based on interpolation coefficients
    for (int32_t i = 0; i < param_.interpolation_number; ++i) {
      double t = static_cast<double>(i) * (dt / static_cast<double>(param_.interpolation_number));
      Eigen::Vector4d t_param(1.0, t, t * t, t * t * t);
      Eigen::Vector4d dt_param(0.0, 1.0, 2.0 * t, 3.0 * t * t);
      Eigen::Vector2d velocity(coeff_x.dot(dt_param), coeff_y.dot(dt_param));
      Pose2d splined_point(coeff_x.dot(t_param), coeff_y.dot(t_param), atan2(velocity(kPoseY), velocity(kPoseX)));
      splined_path.push_back(splined_point);
      splined_path_curvatures.push_back(curvature);
    }
  }
  // Store the final point
  splined_path.push_back(input_path.back());
  splined_path_curvatures.push_back(0.0);
}

// Check for extrema in the spline curve
bool PathInfoCreator::CheckSplinePathExtremum(const Eigen::Vector4d& coeff, const double time) {
  bool has_extremum = false;
  if (fabs(coeff(3)) > std::numeric_limits<double>::epsilon()) {
    double discriminant = coeff(2) * coeff(2) - 3.0 * coeff(3) * coeff(1);
    if (discriminant > 0.0) {
      double pole_small = (-coeff(2) - sqrt(discriminant)) / (3.0 * coeff(3));
      double pole_big = (-coeff(2) + sqrt(discriminant)) / (3.0 * coeff(3));
      if (((pole_small > 0.0) && (pole_small < time)) || ((pole_big > 0.0) && (pole_big < time))) {
        has_extremum = true;
      }
    }
  } else if (fabs(coeff(2)) > std::numeric_limits<double>::epsilon()) {
    double pole = -coeff(1) / (2.0 * coeff(2));
    if ((pole > 0.0) && (pole < time)) {
      has_extremum = true;
    }
  }
  return has_extremum;
}

// Calculate the remaining playback length to the goal at all route points
void PathInfoCreator::CalculateLeftPathLengths(const PoseSeq& path, std::vector<double>& left_path_lengths) {
  left_path_lengths.resize(path.size());
  double sum_length = 0.0;
  left_path_lengths.back() = 0.0;
  for (int i = path.size() - 2; i >= 0; --i) {
    sum_length += (path[i + 1].point() - path[i].point()).norm();
    left_path_lengths[i] = sum_length;
  }
}
}  // namespace tmc_base_path_follower
