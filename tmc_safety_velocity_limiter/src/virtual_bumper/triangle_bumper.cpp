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
/// @file triangle_bumper.hpp
/// @brief Triangular Virtual Bumper
#include "triangle_bumper.hpp"
#include <limits>
#include <angles/angles.h>
#include "obstacle.hpp"
#include "param.hpp"
#include "velocity_slope/velocity_slope.hpp"

namespace {
// ROS Parameter Name
const char* kObstacleSearchDistance ="obstacle_search_distance";  // Target range [m]
const char* kObstacleSearchAngle = "obstacle_search_angle";       // Target angle [rad]
// ROS Parameter Default Value
const double kObstacleSearchDistanceDef = 1.2;  // Target range [m]
const double kObstacleSearchAngleDef = 0.5236;  // Target angle [rad]
}  // anonymous namespace

namespace tmc_safety_velocity_limiter {
/// Constructor
TriangleBumper::TriangleBumper(std::map<std::string, rclcpp::Parameter>& parameters,
    const VelocitySlope::Ptr& velocity_slope) :
    VirtualBumper(parameters, velocity_slope) {
  UpdateParameters(parameters);
}

/// Returns the speed limit ratio based on the distance to the nearest point within range
/// Outputs the coordinates of the point that caused the limitation if limited
/// @param input_velocity [I] Input velocity
/// @param obstacle_pose [O] Outputs the coordinates of the obstacle that caused the limitation
/// @return Limitation ratio (0.0 to 1.0)
double TriangleBumper::LimitVelocityRatio(const Twist& input_velocity,
                                          geometry_msgs::msg::PoseStamped& obstacle_pose) {
  PointCloudPtr obstacle_cloud = Obstacle::GetInstance()->ObstacleCloud();
  double velocity_ratio = 1.0;  // Speed limit ratio
  double distance_ratio = 1.0;  // Ratio of obstacle distance to search distance
  // Get the minimum distance to obstacles within range
  if (FindNearestPoseInRange(obstacle_cloud, input_velocity, obstacle_pose, distance_ratio)) {
    // Calculate speed ratio from the distance to the nearest point
    velocity_ratio = velocity_slope_->CalcRatio(distance_ratio);
  }
  return velocity_ratio;
}

/// Find the point with the shortest distance within range
/// @param input_cloud [I] Point cloud
/// @param input_velocity [I] Movement speed
/// @param nearest_pose [O] Point with the shortest distance
/// @param distance_ratio [O] Ratio of obstacle distance to search distance
/// @return Whether found or not true if found, false if not found
bool TriangleBumper::FindNearestPoseInRange(const PointCloudPtr& input_cloud, const Twist& input_velocity,
                                            geometry_msgs::msg::PoseStamped& nearest_pose, double& distance_ratio) {
  // Determine bumper size ratio based on input speed
  const double bumper_scale = CalcBumperScale(input_velocity);
  if (bumper_scale < std::numeric_limits<double>::epsilon()) {
    return false;
  }
  bool ret = false;
  double velocity_angle = atan2(input_velocity.linear.y, input_velocity.linear.x);
  const double obstacle_search_distance = obstacle_search_distance_ * bumper_scale;
  double min_distance_square = pow(obstacle_search_distance, 2.0);
  for (PointCloud::iterator it = input_cloud->points.begin(); it != input_cloud->points.end(); ++it) {
    // Calculate distance from the center point
    double point_distance_square = (it->x * it->x) + (it->y * it->y);
    if (point_distance_square > min_distance_square) {
      continue;
    }
    // Calculate the angle with the direction of travel
    double point_angle = angles::shortest_angular_distance(atan2(it->y, it->x), velocity_angle);
    point_angle = fabs(point_angle);
    // Determine if it is the nearest point within the search angle
    if (point_angle < obstacle_search_angle_) {
      min_distance_square = point_distance_square;
      nearest_pose.pose.position.x = it->x;
      nearest_pose.pose.position.y = it->y;
      ret = true;
    }
  }
  distance_ratio = sqrt(min_distance_square) / obstacle_search_distance;
  return ret;
}

/// Get ROS PRAM
void TriangleBumper::UpdateParameters(std::map<std::string, rclcpp::Parameter>& parameters) {
  GetOptionalParam(parameters, kObstacleSearchDistance, obstacle_search_distance_,
                                 kObstacleSearchDistanceDef);
  if (obstacle_search_distance_ <= 0.0) {
    RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
        "Parameter [%s] is invalid %lf. Use default value %lf",
        kObstacleSearchDistance, obstacle_search_distance_, kObstacleSearchDistanceDef);
    obstacle_search_distance_ = kObstacleSearchDistanceDef;
  }
  GetOptionalParam(parameters, kObstacleSearchAngle, obstacle_search_angle_, kObstacleSearchAngleDef);
  if (obstacle_search_angle_ <= 0.0) {
    RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
        "Parameter [%s] is invalid %lf. Use default value %lf",
        kObstacleSearchAngle, obstacle_search_angle_, kObstacleSearchAngleDef);
    obstacle_search_angle_ = kObstacleSearchAngleDef;
  }
}
}  // namespace tmc_safety_velocity_limiter
