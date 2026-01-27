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
/// @file occupancy_ellipse_bumper.hpp
/// @brief Elliptical occupancy bumper
#ifndef TMC_SAFETY_VELOCITY_LIMITER_OCCUPANCY_ELLIPSE_BUMPER_HPP_
#define TMC_SAFETY_VELOCITY_LIMITER_OCCUPANCY_ELLIPSE_BUMPER_HPP_
#include <map>
#include <memory>
#include <string>

#include "common.hpp"
#include "virtual_bumper.hpp"

namespace tmc_safety_velocity_limiter {
/// Elliptical occupancy bumper
class OccupancyEllipseBumper : public VirtualBumper {
 public:
  typedef std::shared_ptr<OccupancyEllipseBumper> Ptr;
  OccupancyEllipseBumper(std::map<std::string, rclcpp::Parameter>& parameters, VelocitySlope::Ptr& velocity_slope);

  /// Limit speed according to occupancy values of grids within the search range
  /// If limited, output the coordinates of the obstacle that caused the limitation, which is the coordinate with the largest and closest occupancy value within the range
  /// @param input_velocity [I] Input velocity
  /// @param obstacle_pose [O] Obstacle coordinates. Output the coordinates with the largest and closest occupancy value within the range
  /// @return Limited speed ratio (0.0 to 1.0)
  double LimitVelocityRatio(const Twist& input_velocity, geometry_msgs::msg::PoseStamped& obstacle_pose);

 private:
  /// Calculate the distance from self-position to the farthest point within the elliptical range
  double SearchLongestDistance();

  /// Get ROS PARAM
  void UpdateParameters(std::map<std::string, rclcpp::Parameter>& parameters);

  // Parameters
  double radius_x_;
  double radius_y_;
  double center_position_x_;
  // Maximum distance of search range
  double obstacle_search_distance_;
};
}  // namespace tmc_safety_velocity_limiter
#endif  // TMC_SAFETY_VELOCITY_LIMITER_OCCUPANCY_ELLIPSE_BUMPER_HPP_
