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
#include <tmc_base_path_follower/path_transit_velocity_calculator.hpp>

#include <algorithm>
#include <limits>
#include <vector>

namespace tmc_base_path_follower {
/// Path Passing Speed Calculation
/// @param [I] path_info Path Information
void PathTransitVelocityCalculator::CalculatePathTransitVelocity(const PathInfo& path_info) {
  const std::vector<double> curvatures = path_info.splined_path_curvatures;
  const PoseSeq path = path_info.splined_path;
  const int32_t path_size = path.size();
  transit_velocity_.clear();
  transit_velocity_.resize(path_size);

  // Calculate initial speed and acceleration range for each point
  std::vector<double> max_acceleration;
  std::vector<double> max_deceleration;
  max_acceleration.resize(path_size);
  max_deceleration.resize(path_size);
  for (int32_t i = 0; i < path_size; ++i) {
    if (fabs(curvatures[i]) > std::numeric_limits<double>::epsilon()) {
      // Calculate passing speed based on curvature
      transit_velocity_[i] = std::min<double>(
          param_.max_linear_velocity,
          param_.max_angular_velocity * param_.transit_velocity_angular_velocity_ratio / fabs(curvatures[i]));
      // If below minimum speed, store the minimum speed
      transit_velocity_[i] = std::max<double>(transit_velocity_[i], param_.min_linear_velocity);

      // Calculate acceleration/deceleration based on curvature
      max_acceleration[i] =
          std::min<double>(param_.max_linear_acceleration,
                           param_.max_angular_acceleration / fabs(curvatures[i]));
      max_deceleration[i] =
          std::min<double>(param_.max_linear_deceleration,
                           param_.max_angular_deceleration / fabs(curvatures[i]));
    } else {
      transit_velocity_[i] = param_.max_linear_velocity;
      max_acceleration[i] = param_.max_linear_acceleration;
      max_deceleration[i] = param_.max_linear_deceleration;
    }
  }

  // Verify acceleration range at each point, and adjust speed if out of range
  for (int32_t i = path_size - 2; i > 0; --i) {
    const int32_t prev_index = i - 1;
    // Calculate speed difference between two points
    const double delta_v = transit_velocity_[i] - transit_velocity_[prev_index];

    // Calculate speed change range from maximum acceleration/deceleration
    Point2d prev_point_diff = path[i].point() - path[prev_index].point();
    double delta_l = prev_point_diff.norm();
    double delta_v_max = max_acceleration[prev_index] * delta_l / transit_velocity_[prev_index];
    double delta_v_min = -max_deceleration[prev_index] * delta_l / transit_velocity_[prev_index];
    // Compare speed difference value with possible range, and adjust speed if out of range
    if (delta_v > delta_v_max) {
      // If acceleration is too high, increase subsequent speeds using the maximum acceleration
      for (int32_t fix_index = i; fix_index < path_size - 1; ++fix_index) {
        if (transit_velocity_[fix_index] < transit_velocity_[fix_index - 1] + delta_v_max) {
          // Finish if within the maximum acceleration range from the previous point
          break;
        }
        // Increase speed using maximum acceleration
        transit_velocity_[fix_index] = transit_velocity_[fix_index - 1] + delta_v_max;

        // Calculate maximum acceleration for the next point
        prev_point_diff = path[fix_index + 1].point() - path[fix_index].point();
        delta_l = prev_point_diff.norm();
        delta_v_max = max_acceleration[fix_index + 1] * delta_l / transit_velocity_[fix_index];
      }
    } else if (delta_v < delta_v_min) {
      // If deceleration is too high, decrease speed using the maximum deceleration
      transit_velocity_[prev_index] = (transit_velocity_[i] + sqrt(
          pow(transit_velocity_[i], 2.0) + 4.0 * delta_l * max_deceleration[prev_index])) / 2.0;
    }
  }
}

/// Path Passing Speed Control
/// @param [I] path_index Path Point Index
/// @return Output Speed
double PathTransitVelocityCalculator::GetPathTransitVelocity(const uint32_t path_index) {
  return transit_velocity_[path_index];
}
}  // namespace tmc_base_path_follower
