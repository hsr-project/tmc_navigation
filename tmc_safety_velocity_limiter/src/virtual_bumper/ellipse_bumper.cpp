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
/// @file ellipse_bumper.cpp
/// @brief Elliptical virtual bumper
#include "ellipse_bumper.hpp"
#include <limits>
#include "obstacle.hpp"
#include "param.hpp"
#include "velocity_slope/velocity_slope.hpp"

namespace {
// ROS parameter name
const char* kRadiusX = "radius_x";                   // Radius in the x-axis direction of the restricted area [m]
const char* kRadiusY = "radius_y";                   // Radius in the y-axis direction of the restricted area [m]
const char* kCenterPositionX = "center_position_x";  // x-coordinate of the center of the restricted area [m]
// ROS parameter default value
const double kRadiusXDef = 2.4;          // Radius in the x-axis direction of the restricted area [m]
const double kRadiusYDef = 1.2;          // Radius in the y-axis direction of the restricted area [m]
const double kCenterPositionXDef = 1.9;  // x-coordinate of the center of the restricted area [m]
}  // anonymous namespace

namespace tmc_safety_velocity_limiter {
/// Constructor
/// @param parameters [I] Parameters
/// @param velocity_slope [I] Velocity slope
EllipseBumper::EllipseBumper(std::map<std::string, rclcpp::Parameter>& parameters,
    const VelocitySlope::Ptr& velocity_slope) :
    VirtualBumper(parameters, velocity_slope) {
  UpdateParameters(parameters);
  // Get the maximum distance within the range
  obstacle_search_distance_ = SearchLongestDistance();
}

/// Return the speed limit ratio according to the distance to the nearest point within the range
/// Output the coordinates of the obstacle that caused the restriction when restricted
/// @param input_velocity [I] Input velocity
/// @param obstacle_pose [O] Obstacle coordinates
/// @return Speed limit ratio (0.0 to 1.0)
double EllipseBumper::LimitVelocityRatio(const Twist& input_velocity,
    geometry_msgs::msg::PoseStamped& obstacle_pose) {
  PointCloudPtr obstacle_cloud = Obstacle::GetInstance()->ObstacleCloud();
  double velocity_ratio = 1.0;  // Speed limit ratio
  double distance_ratio = 1.0;  // Ratio of obstacle distance to search distance
  // Get the point with the shortest distance within the range
  if (FindNearestPoseInRange(obstacle_cloud, input_velocity, obstacle_pose, distance_ratio)) {
    // Calculate the speed ratio from the ratio of obstacle distance to search distance
    velocity_ratio = velocity_slope_->CalcRatio(distance_ratio);
  }
  return velocity_ratio;
}

/// Find the point with the shortest distance within the range
/// @param input_cloud [I] Point cloud
/// @param input_velocity [I] Moving speed
/// @param nearest_pose [O] Point with the shortest distance
/// @return Whether found or not true found false not found
bool EllipseBumper::FindNearestPoseInRange(const PointCloudPtr& input_cloud, const Twist& input_velocity,
                                           geometry_msgs::msg::PoseStamped& nearest_pose, double& distance_ratio) {
  // Determine the bumper size ratio according to the input speed
  const double bumper_scale = CalcBumperScale(input_velocity);
  if (bumper_scale < std::numeric_limits<double>::epsilon()) {
    return false;
  }
  bool ret = false;
  const double obstacle_search_distance = obstacle_search_distance_ * bumper_scale;
  double min_distance_square = pow(obstacle_search_distance, 2.0);
  const double velocity_angle = atan2(input_velocity.linear.y, input_velocity.linear.x);
  // Rotate the obstacle coordinates by -velocity_angle to make the orientation of the search range ellipse based on the direction of travel
  const double rotate_sin = sin(-velocity_angle);
  const double rotate_cos = cos(-velocity_angle);
  for (PointCloud::iterator it = input_cloud->points.begin(); it != input_cloud->points.end(); ++it) {
    const double x = it->x * rotate_cos - it->y * rotate_sin;
    const double y = it->x * rotate_sin + it->y * rotate_cos;
    const double clue_for_ellipse_range = pow((x - center_position_x_ * bumper_scale), 2.0) /
                                          pow((radius_x_ * bumper_scale), 2.0) +
                                          pow(y, 2.0) / pow((radius_y_ * bumper_scale), 2.0);
    // Calculate whether the object exists within the elliptical speed limit range
    if (clue_for_ellipse_range < 1) {
      // Calculate the distance from self-position
      const double point_distance_square = it->x * it->x + it->y * it->y;
      // Determine if it is the nearest point within the search range
      if (point_distance_square < min_distance_square) {
        min_distance_square = point_distance_square;
        nearest_pose.pose.position.x = it->x;
        nearest_pose.pose.position.y = it->y;
        ret = true;
      }
    }
  }
  distance_ratio = sqrt(min_distance_square) / obstacle_search_distance;
  return ret;
}

/// Find the distance from self-position to the farthest point within the elliptical range
double EllipseBumper::SearchLongestDistance() {
  double longest_distance = DBL_MAX;
  // Calculate the distance from the origin (self-position) to the front, side, and rear ends of the ellipse
  double ellipse_top = fabs(radius_x_ + center_position_x_);
  double ellipse_side = sqrt(center_position_x_ * center_position_x_ + radius_y_ * radius_y_);
  double ellipse_bottom = fabs(radius_x_ - center_position_x_);
  // Choose the farthest one among the three points found
  if (ellipse_top > ellipse_side && ellipse_top > ellipse_bottom) {
    longest_distance = ellipse_top;
  } else if (ellipse_side > ellipse_top && ellipse_side > ellipse_bottom) {
    longest_distance = ellipse_side;
  } else {
    longest_distance = ellipse_bottom;
  }
  return longest_distance;
}

/// Get ROS PRAM
void EllipseBumper::UpdateParameters(std::map<std::string, rclcpp::Parameter>& parameters) {
  GetOptionalParam(parameters, kRadiusX, radius_x_, kRadiusXDef);
  if (radius_x_ <= 0.0) {
    RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
        "Parameter [%s] is invalid %lf. Use default value %lf", kRadiusX, radius_x_, kRadiusXDef);
    radius_x_ = kRadiusXDef;
  }
  GetOptionalParam(parameters, kRadiusY, radius_y_, kRadiusYDef);
  if (radius_y_ <= 0.0) {
    RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
        "Parameter [%s] is invalid %lf. Use default value %lf", kRadiusY, radius_y_, kRadiusYDef);
    radius_y_ = kRadiusYDef;
  }
  GetOptionalParam(parameters, kCenterPositionX, center_position_x_, kCenterPositionXDef);
}
}  // namespace tmc_safety_velocity_limiter
