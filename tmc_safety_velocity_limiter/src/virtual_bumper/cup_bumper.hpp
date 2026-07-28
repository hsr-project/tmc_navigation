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
/// @file cup_bumper.hpp
/// @brief Cup-shaped virtual bumper
#ifndef TMC_SAFETY_VELOCITY_LIMITER_CUP_BUMPER_HPP_
#define TMC_SAFETY_VELOCITY_LIMITER_CUP_BUMPER_HPP_
#include <map>
#include <memory>
#include <string>

#include "common.hpp"
#include "virtual_bumper.hpp"

namespace tmc_safety_velocity_limiter {
/// Cup-shaped virtual bumper
class CupBumper : public VirtualBumper {
 public:
  typedef std::shared_ptr<CupBumper> Ptr;
  CupBumper(std::map<std::string, rclcpp::Parameter>& parameters, VelocitySlope::Ptr& velocity_slope);

  /// Returns the speed limit ratio based on the distance to the nearest point within the range.
  /// Outputs the coordinates of the obstacle that caused the limitation when restricted.
  /// @param input_velocity [I] Input velocity
  /// @param obstacle_pose [O] Outputs obstacle coordinates
  /// @return Limitation ratio (0.0 to 1.0)
  double LimitVelocityRatio(const Twist& input_velocity, geometry_msgs::msg::PoseStamped& obstacle_pose);

 private:
  /// Finds the point with the shortest distance within the range.
  /// @param input_cloud [I] Point cloud
  /// @param input_velocity [I] Movement velocity
  /// @param nearest_pose [O] Point with the shortest distance
  /// @param distance_ratio [O] Ratio of obstacle distance to search distance
  /// @return Whether found or not: true if found, false if not found
  bool FindNearestPoseInRange(const PointCloudPtr& input_cloud, const Twist& input_velocity,
                              geometry_msgs::msg::PoseStamped& nearest_pose, double& distance_ratio);

  /// Retrieve ROS PARAM
  void UpdateParameters(std::map<std::string, rclcpp::Parameter>& parameters);

  double bottom_length_;
  double obstacle_search_angle_;
  double obstacle_search_distance_;
  double robot_radius_;
};
}  // namespace tmc_safety_velocity_limiter
#endif  // TMC_SAFETY_VELOCITY_LIMITER_CUP_BUMPER_HPP_
