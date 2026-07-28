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
// Utility functions for ROS-dependent tests
#ifndef TMC_BASE_PATH_PLANNER_TEST_UTILS_ROS_HPP_
#define TMC_BASE_PATH_PLANNER_TEST_UTILS_ROS_HPP_

#include <limits>
#include <memory>
#include <string>

#include <angles/angles.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tmc_base_path_planner {
// Check if two points match
bool IsMatchPoseStamped(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2) {
  const double yaw1 = tf2::getYaw(pose1.pose.orientation);
  const double yaw2 = tf2::getYaw(pose2.pose.orientation);
  if (fabs(pose1.pose.position.x - pose2.pose.position.x) > std::numeric_limits<double>::epsilon() ||
      fabs(pose1.pose.position.y - pose2.pose.position.y) > std::numeric_limits<double>::epsilon() ||
      fabs(angles::shortest_angular_distance(yaw1, yaw2)) > std::numeric_limits<double>::epsilon()) {
    return false;
  }
  return true;
}

// Minimum distance from a point to a path
double DistancePointToPath(const geometry_msgs::msg::Point& point, const nav_msgs::msg::Path& path) {
  double min_distance = std::numeric_limits<double>::max();
  for (uint32_t i = 0; i < path.poses.size(); ++i) {
    const double distance = sqrt(pow(path.poses[i].pose.position.x - point.x, 2.0) +
                                 pow(path.poses[i].pose.position.y - point.y, 2.0));
    if (min_distance > distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

// Draw circular obstacles on the map
void DrawObstacleCircle(nav_msgs::msg::OccupancyGrid& map, const geometry_msgs::msg::Point& center,
    const double radius) {
  for (uint32_t ih = 0; ih < map.info.height; ++ih) {
    const double y = ih * map.info.resolution;
    for (uint32_t iw = 0; iw < map.info.width; ++iw) {
      const double x = iw * map.info.resolution;
      // Calculate the distance from the grid to the center, subtracting 1 grid to include the grid on the circumference as part of the obstacle range
      const double distance = sqrt(pow(center.x - x, 2.0) + pow(center.y - y, 2.0)) - map.info.resolution;
      if (distance < radius) {
        // Store the maximum occupancy probability within the range
        map.data[iw + ih * map.info.width] = 100;
      }
    }
  }
}

geometry_msgs::msg::Quaternion CreateQuaternionMsgFromYaw(const double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

/// Pose generation
geometry_msgs::msg::Pose CreatePose(const double x, const double y, const double yaw) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation = CreateQuaternionMsgFromYaw(0.0);
  return pose;
}

/// Generate a map with the origin at (0,0), no rotation, and all areas free
nav_msgs::msg::OccupancyGrid CreateFreeMap(const uint32_t width, const uint32_t height, const double resolution) {
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = width;
  map.info.height = height;
  map.info.resolution = resolution;
  map.info.origin = CreatePose(0.0, 0.0, 0.0);
  map.data.resize(map.info.width * map.info.height, 0);
  return map;
}

// TODO(syuuhei_shiro): tmc_rostest_utilをROS2化してそこに置く
// Load parameters from a yaml file
void LoadParameterFromYaml(std::shared_ptr<rclcpp::Node> node,
    const std::string& yaml_directory, const std::string& yaml_name) {
  const std::string yaml_path = yaml_directory + yaml_name;
  // Load yaml and generate a ParameterMap
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_params_t* yaml_params = rcl_yaml_node_struct_init(allocator);
  rcl_parse_yaml_file(yaml_path.c_str(), yaml_params);
  rclcpp::ParameterMap yaml_param_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);
  // Set ROS parameters to the node
  const std::string parameter_space = "/" + std::string(node->get_name());
  auto iter = yaml_param_map.find(parameter_space);
  for (auto& param : iter->second) {
    node->declare_parameter(param.get_name(), param.get_type());
    node->set_parameter(param);
  }
}
}  // namespace tmc_base_path_planner
#endif  // TMC_BASE_PATH_PLANNER_TEST_UTILS_HPP_
