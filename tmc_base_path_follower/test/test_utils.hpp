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
// Test utility function group
#ifndef TMC_BASE_PATH_FOLLOWER_TEST_UTILS_HPP_
#define TMC_BASE_PATH_FOLLOWER_TEST_UTILS_HPP_
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <nav_msgs/msg/path.hpp>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_base_path_follower/common.hpp>
#include <tmc_base_path_follower/parameter_default_value.hpp>
#include <tmc_pose_2d_lib/ros_if.hpp>

namespace tmc_base_path_follower {
// Not the true epsilon, but has some width for boundary value testing
constexpr double kEpsilon = 0.0001;

// TODO(syuuhei_shiro): tmc_rostest_utilをROS2に対応させてそちらを参照する
using WaitFunctionType = std::function<bool()>;

/**
 * @brief Wait until some condition is met
 *
 * @param condition_function Condition function
 * @param timeout_sec Maximum wait time (sec)
 * @param rate_hz Check cycle (hz) Default 100.0 (hz)
 *
 * @return Condition met or not met
 */
bool WaitUntil(std::vector<rclcpp::Node::SharedPtr> nodes,
    WaitFunctionType condition_function, double timeout_sec, double rate_hz = 100.0) {
  // Error check for arguments
  if (!condition_function) {
    throw std::invalid_argument("Function for waiting is empty.");
  }
  if (timeout_sec < 0.0) {
    throw std::invalid_argument("Timeout must must have fully value");
  }
  if (rate_hz < std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument("Rate to validate must have fully value");
  }

  const rclcpp::Time end_time = rclcpp::Clock(RCL_ROS_TIME).now() + rclcpp::Duration::from_seconds(timeout_sec);
  rclcpp::Rate rate(rate_hz);
  while (rclcpp::ok()) {
    for (auto node : nodes) {
      rclcpp::spin_some(node);
    }
    if (condition_function()) return true;

    if (rclcpp::Clock(RCL_ROS_TIME).now() >= end_time) break;

    rate.sleep();
  }
  return false;
}

// TODO(syuuhei_shiro): tmc_rostest_utilをROS2化してそこに置く
// Load parameters from yaml file
void LoadParameterFromYaml(std::shared_ptr<rclcpp::Node> node,
    const std::string& yaml_directory, const std::string& yaml_name) {
  const std::string yaml_path = yaml_directory + yaml_name;
  // Load yaml and generate ParameterMap
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_params_t* yaml_params = rcl_yaml_node_struct_init(allocator);
  rcl_parse_yaml_file(yaml_path.c_str(), yaml_params);
  rclcpp::ParameterMap yaml_param_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);
  // Set ros parameters to node
  const std::string parameter_space = "/" + std::string(node->get_name());
  auto iter = yaml_param_map.find(parameter_space);
  for (auto& param : iter->second) {
    node->declare_parameter(param.get_name(), param.get_type());
    node->set_parameter(param);
  }
}

/// Generate a straight path (Pose2d)
PoseSeq CreateLinearPath(const Pose2d& start_pose, const Pose2d& goal_pose, const double interval) {
  // Calculate points
  const Point2d position_error = goal_pose.point() - start_pose.point();
  const double path_length = position_error.norm();
  const uint32_t path_num = static_cast<uint32_t>(path_length / interval);

  // Calculate direction of the line
  double path_direction = atan2(position_error.y(), position_error.x());

  // Store path points
  PoseSeq output_path;
  output_path.push_back(start_pose);
  for (uint32_t i = 1; i < path_num; ++i) {
    const double x = start_pose.x() + i * interval * cos(path_direction);
    const double y = start_pose.y() + i * interval * sin(path_direction);
    Pose2d path_pose(x, y, path_direction);
    output_path.push_back(path_pose);
  }
  // Store goal
  output_path.push_back(goal_pose);
  return output_path;
}

/// Generate a straight path (nav_msg::Path)
nav_msgs::msg::Path CreateLinearPath(const geometry_msgs::msg::PoseStamped& start_pose,
                                const geometry_msgs::msg::PoseStamped& goal_pose,
                                const double interval) {
  const Pose2d start_pose2d = tmc_pose_2d_lib::GetPose2dFromRosMsg(start_pose);
  const Pose2d goal_pose2d = tmc_pose_2d_lib::GetPose2dFromRosMsg(goal_pose);
  const PoseSeq path_pose2d = CreateLinearPath(start_pose2d, goal_pose2d, interval);

  nav_msgs::msg::Path output_path;
  output_path.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  for (const Pose2d& path_pose_pose2d : path_pose2d) {
    geometry_msgs::msg::PoseStamped path_pose;
    path_pose.pose = tmc_pose_2d_lib::GetPoseMsg(path_pose_pose2d);
    output_path.poses.push_back(path_pose);
  }

  return output_path;
}

/// Calculate distance between two points
double CalcDistance(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2) {
  double diff_x = pose1.pose.position.x - pose2.pose.position.x;
  double diff_y = pose1.pose.position.y - pose2.pose.position.y;
  return sqrt(diff_x * diff_x + diff_y * diff_y);
}

/// Get nearest point on the path
uint32_t NearestIndex(const geometry_msgs::msg::PoseStamped& pose, const nav_msgs::msg::Path& path) {
  uint32_t nearest_index = 0;
  double error_from_path = std::numeric_limits<double>::max();
  for (uint32_t i = 0; i < path.poses.size(); ++i) {
    const double linear_error = CalcDistance(path.poses[i], pose);
    if (linear_error < error_from_path) {
      nearest_index = i;
      error_from_path = linear_error;
    }
  }
  return nearest_index;
}

/// Calculate distance to nearest point on the path
double DistancePoseToPath(const geometry_msgs::msg::PoseStamped& pose, const nav_msgs::msg::Path& path) {
  const uint32_t nearest_index = NearestIndex(pose, path);
  return CalcDistance(path.poses[nearest_index], pose);
}

// Return a position slightly inside the goal area reach condition
Pose2d ArriveGoalAreaPose(const Pose2d& goal) {
  // Set X coordinate slightly inside the goal line, Y coordinate slightly inside the threshold radius
  // Set angle to not judge goal reach
  const double diff_x = -kGoalLineLengthDefault + kEpsilon;
  const double diff_y = sqrt(kGoalAreaLengthDefault * kGoalAreaLengthDefault - diff_x * diff_x) - kEpsilon;
  const Pose2d pose(
      goal.x() + diff_x,
      goal.y() + diff_y,
      goal.theta() + kGoalStopErrorAngleDefault + kEpsilon);
  return pose;
}

// Return a position slightly outside the goal line
Pose2d AheadGoalLinePose(const Pose2d& goal) {
  // Set X coordinate slightly outside the goal line, Y coordinate slightly inside the threshold radius
  // Set angle to not judge goal reach
  const double diff_x = -kGoalLineLengthDefault - kEpsilon;
  const double diff_y = sqrt(kGoalAreaLengthDefault * kGoalAreaLengthDefault - diff_x * diff_x) - kEpsilon;
  const Pose2d pose(
      goal.x() + diff_x,
      goal.y() + diff_y,
      goal.theta() + kGoalStopErrorAngleDefault + kEpsilon);
  return pose;
}

// Return a position slightly outside the goal area
Pose2d OutsideGoalAreaPose(const Pose2d& goal) {
  // Set X coordinate slightly inside the goal line, Y coordinate slightly outside the threshold radius
  // Set angle to not judge goal reach
  const double diff_x = -kGoalLineLengthDefault + kEpsilon;
  const double diff_y = sqrt(kGoalAreaLengthDefault * kGoalAreaLengthDefault - diff_x * diff_x) + kEpsilon;
  const Pose2d pose(
      goal.x() + diff_x,
      goal.y() + diff_y,
      goal.theta() + kGoalStopErrorAngleDefault + kEpsilon);
  return pose;
}

// Return a position slightly inside the goal reach condition
Pose2d ArriveGoalPose(const Pose2d& goal) {
  // Set angle and coordinates slightly inside the threshold
  const Pose2d pose(
      goal.x() + kGoalStopErrorLengthDefault / sqrt(2.0) - kEpsilon,
      goal.y() + kGoalStopErrorLengthDefault / sqrt(2.0) - kEpsilon,
      goal.theta() + kGoalStopErrorAngleDefault - kEpsilon);
  return pose;
}

// Return a position with angle slightly deviated from goal reach condition
Pose2d NotArrivedGoalAngularPose(const Pose2d& goal) {
  // Set coordinates slightly inside the threshold, angle slightly outside the threshold
  const Pose2d pose(
      goal.x() + kGoalStopErrorLengthDefault / sqrt(2.0) - kEpsilon,
      goal.y() + kGoalStopErrorLengthDefault / sqrt(2.0) - kEpsilon,
      goal.theta() + kGoalStopErrorAngleDefault + kEpsilon);
  return pose;
}

// Return a position with coordinates slightly deviated from goal reach condition
Pose2d NotArrivedGoalLinerPose(const Pose2d& goal) {
  // Set coordinates slightly outside the threshold, angle slightly inside the threshold
  const Pose2d pose(
      goal.x() + kGoalStopErrorLengthDefault / sqrt(2.0) + kEpsilon,
      goal.y() + kGoalStopErrorLengthDefault / sqrt(2.0) + kEpsilon,
      goal.theta() + kGoalStopErrorAngleDefault - kEpsilon);
  return pose;
}

/// Generate an arc-shaped path
/// radius: [I] Radius of the circle [m]
/// initial_pose: [I] Coordinates of the arc's starting point
/// path_interval: [I] Path point interval
/// arc_angle: [I] Arc angle [rad] Positive for counterclockwise, negative for clockwise
/// path: [O] Output path
void CreateArcPath(const double radius, const Pose2d& initial_pose, const double path_interval,
                   const double arc_angle, PoseSeq& path) {
  if (fabs(arc_angle) < std::numeric_limits<double>::epsilon()) {
    return;
  }
  const double arc_angle_sign = arc_angle / fabs(arc_angle);
  // Determine the center of the circle from the starting point and radius
  const Pose2d center(initial_pose.x() - sin(initial_pose.theta()) * arc_angle_sign * radius,
                      initial_pose.y() + cos(initial_pose.theta()) * arc_angle_sign * radius, 0.0);
  // Number of points on the arc: Arc length / Path point interval + 1 for the goal point
  const int32_t points =
      static_cast<int32_t>((std::abs(arc_angle) * radius) / path_interval) + 1;
  for (int32_t i = 0; i < points; ++i) {
    // Direction of path points
    const double path_theta =
        initial_pose.theta() + (arc_angle / static_cast<double>(points - 1)) * static_cast<double>(i);
    // Angle from the center of the circle to the path points
    const double theta = path_theta - arc_angle_sign * M_PI / 2.0;
    path.push_back(Pose2d(center.x() + cos(theta) * radius,
                          center.y() + sin(theta) * radius,
                          angles::normalize_angle(path_theta)));
  }
}
}  // namespace tmc_base_path_follower
#endif  // TMC_BASE_PATH_FOLLOWER_TEST_UTILS_HPP_
