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
/// @file cup_bumper.cpp
/// @brief Cup-shaped virtual bumper
#include "cup_bumper.hpp"
#include <limits>
#include <angles/angles.h>
#include <geometry_msgs/msg/point.hpp>
#include "obstacle.hpp"
#include "param.hpp"
#include "velocity_slope/velocity_slope.hpp"

namespace {
// ROS parameter name
const char* kBottomLength = "bottom_length";                       // Cup bottom length [m]
const char* kObstacleSearchDistance = "obstacle_search_distance";  // Restricted range [m]
const char* kObstacleSearchAngle = "obstacle_search_angle";        // Restricted angle [rad]
const char* kRobotRadius = "robot_radius";                         // Robot radius [m]
// ROS parameter default values
const double kBottomLengthDef = 0.44;            // Cup bottom length [m]
const double kObstacleSearchDistanceDef = 1.2;   // Restricted range [m]
const double kObstacleSearchAngleDef = 0.1745;   // Restricted angle [rad]
const double kRobotRadiusDef = 0.22;             // Robot radius [m]
}  // anonymous namespace

namespace tmc_safety_velocity_limiter {
/// Constructor
CupBumper::CupBumper(std::map<std::string, rclcpp::Parameter>& parameters, VelocitySlope::Ptr& velocity_slope) :
    VirtualBumper(parameters, velocity_slope) {
  UpdateParameters(parameters);
}

/// Returns the velocity restriction ratio based on the distance to the nearest point within the range.
/// Outputs the coordinates of the obstacle that caused the restriction if restricted.
/// @param input_velocity [I] Input velocity
/// @param obstacle_pose [O] Outputs obstacle coordinates
/// @return Restriction ratio (0.0 to 1.0)
double CupBumper::LimitVelocityRatio(const Twist& input_velocity, geometry_msgs::msg::PoseStamped& obstacle_pose) {
  PointCloudPtr obstacle_cloud = Obstacle::GetInstance()->ObstacleCloud();
  double velocity_ratio = 1.0;  // Velocity restriction ratio
  double distance_ratio = 1.0;  // Ratio of obstacle distance to search distance
  // Retrieve the nearest point within the range
  if (FindNearestPoseInRange(obstacle_cloud, input_velocity, obstacle_pose, distance_ratio)) {
    // Calculate the velocity restriction ratio based on the ratio of obstacle distance to search distance
    velocity_ratio = velocity_slope_->CalcRatio(distance_ratio);
  }
  return velocity_ratio;
}

/// Find the nearest point within the range
/// @param input_cloud [I] Point cloud
/// @param input_velocity [I] Movement velocity
/// @param nearest_pose [O] Nearest point
/// @param distance_ratio [O] Ratio of obstacle distance to search distance
/// @return Whether found or not: true if found, false if not found
bool CupBumper::FindNearestPoseInRange(const PointCloudPtr& input_cloud, const Twist& input_velocity,
    geometry_msgs::msg::PoseStamped& nearest_pose, double& distance_ratio) {
  // Determine bumper size ratio based on input velocity
  const double bumper_scale = CalcBumperScale(input_velocity);
  if (bumper_scale < std::numeric_limits<double>::epsilon()) {
    return false;
  }
  bool ret = false;
  const double velocity_angle = atan2(input_velocity.linear.y, input_velocity.linear.x);
  // Calculate the vertices of the cup's bottom edge
  // Angle and distance from the robot position to the bottom edge vertices
  const double theta = atan2(bottom_length_ / 2.0, -robot_radius_);
  const double apex_distance = sqrt(pow(bottom_length_ / 2.0, 2.0) + pow(robot_radius_, 2.0));
  // Cup bottom edge left vertex
  geometry_msgs::msg::Point apex_left;
  apex_left.x = cos(velocity_angle + theta) * apex_distance;
  apex_left.y = sin(velocity_angle + theta) * apex_distance;
  // Cup bottom edge right vertex
  geometry_msgs::msg::Point apex_right;
  apex_right.x = cos(velocity_angle - theta) * apex_distance;
  apex_right.y = sin(velocity_angle - theta) * apex_distance;

  const double obstacle_search_distance = obstacle_search_distance_ * bumper_scale;
  double min_distance_square = pow(obstacle_search_distance, 2.0);
  for (PointCloud::iterator it = input_cloud->points.begin(); it != input_cloud->points.end(); ++it) {
    // Distance from the robot origin to the point
    const double point_distance_square = it->x * it->x + it->y * it->y;
    if (point_distance_square > min_distance_square) {
      continue;
    }
    // Calculate the angle from the left and right vertices of the cup's bottom edge to the point
    const double point_angle_left =
        angles::shortest_angular_distance(velocity_angle, atan2(it->y - apex_left.y, it->x - apex_left.x));
    const double point_angle_right =
        angles::shortest_angular_distance(velocity_angle, atan2(it->y - apex_right.y, it->x - apex_right.x));
    // Determine if within the cup range
    if (point_angle_left < obstacle_search_angle_ / 2.0 &&
        point_angle_left > - M_PI / 2.0 &&
        point_angle_right > -obstacle_search_angle_ / 2.0 &&
        point_angle_right < M_PI / 2.0) {
      min_distance_square = point_distance_square;
      nearest_pose.pose.position.x = it->x;
      nearest_pose.pose.position.y = it->y;
      ret = true;
    }
  }
  distance_ratio = sqrt(min_distance_square) / obstacle_search_distance;
  return ret;
}

/// Retrieve ROS PARAM
void CupBumper::UpdateParameters(std::map<std::string, rclcpp::Parameter>& parameters) {
  GetOptionalParam(parameters, kBottomLength, bottom_length_, kBottomLengthDef);
  if (bottom_length_ <= 0.0) {
    RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
        "Parameter [%s] is invalid %lf. Use default value %lf",
        kBottomLength, bottom_length_, kBottomLengthDef);
    bottom_length_ = kBottomLengthDef;
  }

  GetOptionalParam(parameters, kObstacleSearchDistance, obstacle_search_distance_, kObstacleSearchDistanceDef);
  if (obstacle_search_distance_ <= 0.0) {
    RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
        "Parameter [%s] is invalid %lf. Use default value %lf",
        kObstacleSearchDistance, obstacle_search_distance_, kObstacleSearchDistanceDef);
    obstacle_search_distance_ = kObstacleSearchDistanceDef;
  }

  GetOptionalParam(parameters, kObstacleSearchAngle, obstacle_search_angle_, kObstacleSearchAngleDef);
  if (obstacle_search_angle_ < 0.0) {
    RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
        "Parameter [%s] is invalid %lf. Use default value %lf",
        kObstacleSearchAngle, obstacle_search_angle_, kObstacleSearchAngleDef);
    obstacle_search_angle_ = kObstacleSearchAngleDef;
  }
  GetOptionalParam(parameters, kRobotRadius, robot_radius_, kRobotRadiusDef);
  if (robot_radius_ <= 0.0) {
    RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
        "Parameter [%s] is invalid %lf. Use default value %lf",
        kRobotRadius, robot_radius_, kRobotRadiusDef);
    robot_radius_ = kRobotRadiusDef;
  }
}
}  // namespace tmc_safety_velocity_limiter
