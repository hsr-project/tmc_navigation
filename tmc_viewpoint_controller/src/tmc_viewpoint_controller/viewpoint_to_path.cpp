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
/// @file viewpoint_to_path.cpp
/// @brief Calculate the angle to direct the viewpoint towards the destination
#include <tmc_viewpoint_controller/viewpoint_to_path.hpp>

#include <algorithm>
#include <functional>
#include <string>
#include <utility>
#include <vector>
#include <angles/angles.h>

#include <tmc_viewpoint_controller/param.hpp>

namespace {
using std::placeholders::_1;
/// Path topic name
const char* const kPathTopicName = "base_local_path";
/// Default value of focus_path_length [m]
const double kDefaultFocusPathLength = 0.5;
/// Allowable distance [m] between the robot and the nearest path point
const double kMaxNearestDist = 1.0;
/// Allowable distance [m] from the target viewpoint position to the path point
const double kMaxTargetError = 1.0;
/// Length of a valid path [number of points]
const uint32_t kValidPathNum = 1;

typedef std::pair<int32_t, double> IndexWithDistance;

double Distance(const double x1, const double y1, const double x2, const double y2) {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
/// Function object to calculate the distance from the robot
class RobotToPathDistance : public std::binary_function<Eigen::Vector3d, geometry_msgs::msg::PoseStamped, double> {
 public:
  double operator()(Eigen::Vector3d robot, geometry_msgs::msg::PoseStamped path) {
    return Distance(robot(0), robot(1), path.pose.position.x, path.pose.position.y);
  }
};

/// Nearest path point calculation
/// @param [in] robot_pose Robot's self-position
/// @param [in] path Path information
/// @return IndexWithDistance Index of the nearest path point and the distance to the path point
IndexWithDistance NearestPathPoint(const Eigen::Vector3d& robot_pose, const nav_msgs::msg::Path& path) {
  IndexWithDistance nearest_path_info;
  std::vector<double> dist;
  // Find the nearest path point from the robot
  std::transform(path.poses.begin(), path.poses.end(), std::back_inserter(dist),
                 std::bind(RobotToPathDistance(), robot_pose, _1));
  // Index of the minimum value
  std::vector<double>::iterator min = std::min_element(dist.begin(), dist.end());
  nearest_path_info.second = *min;
  nearest_path_info.first = std::distance(dist.begin(), min);
  return nearest_path_info;
}

/// Target path point calculation
/// @param [in] path Path information
/// @param [in] nearest_path_index Index of the nearest path point
/// @param [in] focus_path_length Distance to the focus point
/// @return IndexWithDistance Index of the path point nearest to the focus point and the distance to the path point
IndexWithDistance TargetPathPoint(const nav_msgs::msg::Path& path, const int32_t nearest_path_index,
                                  const double focus_path_length) {
  IndexWithDistance target_path_info;
  double pair_dist = 0.0;
  double path_length = 0.0;
  std::vector<double> error_dist;
  target_path_info.first = 0;
  target_path_info.second = 0.0;

  for (std::vector<geometry_msgs::msg::PoseStamped>::const_iterator it = path.poses.begin() + nearest_path_index + 1;
       it != path.poses.end(); ++it) {
    // Calculate the distance between two points
    pair_dist =
        Distance(it->pose.position.x, it->pose.position.y, (it - 1)->pose.position.x, (it - 1)->pose.position.y);
    // Accumulate the distance between two points to calculate the path length
    path_length += pair_dist;
    // Store the deviation between the focus point and the path length
    error_dist.push_back(fabs(focus_path_length - path_length));
  }
  // When the focus point is within the path
  if (path_length > focus_path_length) {
    // Find the path point closest to the focus point
    std::vector<double>::iterator min = std::min_element(error_dist.begin(), error_dist.end());
    target_path_info.first = std::distance(error_dist.begin(), min) + nearest_path_index + 1;
    target_path_info.second = *min;
  }
  return target_path_info;
}
}  // end anonymous namespace


namespace tmc_viewpoint_controller {
using std::placeholders::_1;
/// Constructor
ViewpointToPath::ViewpointToPath(const rclcpp::Node::SharedPtr node)
    : focus_path_length_(0.0) {
  // Parameter acquisition process
  // Focus path point
  GetOptionalParam(node, "focus_path_length", focus_path_length_, kDefaultFocusPathLength);
  // Path Subscriber setup
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      kPathTopicName, 1, std::bind(&ViewpointToPath::PathCallback, this, _1));
}

/// Destructor
ViewpointToPath::~ViewpointToPath() {}

/// Path direction viewpoint calculation
bool ViewpointToPath::ViewPointToPathDirection(const Eigen::Vector3d& robot_pose, double& out_direction) {
  if (path_.poses.empty()) {
    return false;
  }
  IndexWithDistance nearest_path_info;    // (Nearest point, distance to the nearest point)
  // Calculate the nearest point on the path
  nearest_path_info = NearestPathPoint(robot_pose, path_);
  // If the distance to the nearest point exceeds a certain threshold, do not control the viewpoint
  if (nearest_path_info.second > kMaxNearestDist) {
    RCLCPP_DEBUG(rclcpp::get_logger("view_point_controller"), "Exceed max nearest distance.");
    RCLCPP_DEBUG(rclcpp::get_logger("view_point_controller"), "No Need to View-Control");
    return false;
  }

  // Calculate the target point to direct the viewpoint
  IndexWithDistance target_path_info;    // (Target point, target position error)
  target_path_info = TargetPathPoint(path_, nearest_path_info.first, focus_path_length_);
  if (target_path_info.second > kMaxTargetError) {
    RCLCPP_DEBUG(rclcpp::get_logger("view_point_controller"), "Exceed max target error");
    RCLCPP_DEBUG(rclcpp::get_logger("view_point_controller"), "No Need to View-Control");
    return false;
  }
  // Global viewpoint
  double view_direction = 0.0;
  // Neck rotation amount
  double robot_view_direction = 0.0;
  if (target_path_info.first == 0) {
    // When the path becomes shorter (approaching the goal), return the viewpoint to the fixed position (forward)
    robot_view_direction = 0.0;
    view_direction = -robot_pose(2);
    RCLCPP_DEBUG(rclcpp::get_logger("view_point_controller"),
        "short path %d", static_cast<uint32_t>(path_.poses.size()));
    RCLCPP_DEBUG(rclcpp::get_logger("view_point_controller"), "No target_index, so set default position");
    path_.poses.clear();
  } else {
    // Calculate the viewpoint angle from the target point
    Eigen::Vector2d next_target_position;
    Eigen::Vector2d target_position;
    target_position << path_.poses[target_path_info.first].pose.position.x,
        path_.poses[target_path_info.first].pose.position.y;
    next_target_position << path_.poses[target_path_info.first - 1].pose.position.x,
        path_.poses[target_path_info.first - 1].pose.position.y;
    view_direction = atan2(target_position(1) - next_target_position(1), target_position(0) - next_target_position(0));
    // Calculate the neck pan angle from the viewpoint angle
    robot_view_direction = angles::normalize_angle(view_direction - robot_pose(2));
  }
  out_direction = robot_view_direction;
  return true;
}

/// Path acquisition callback function
void ViewpointToPath::PathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  if (msg->poses.size() >= kValidPathNum) {
    path_ = *msg;
  } else {
    path_.poses.clear();
  }
}
}  // end namespace tmc_viewpoint_controller

