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
// Utility functions group for tmc_base_velocity_adjuster automatic testing
#ifndef TMC_BASE_VELOCITY_ADJUSTER_TEST_UTILS_HPP_
#define TMC_BASE_VELOCITY_ADJUSTER_TEST_UTILS_HPP_
#include <limits>
#include <memory>
#include <string>
#include <Eigen/Core>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>


namespace { // NOLINT
// Minimum value for boundary value analysis
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
}  // anonymous namespace

namespace tmc_base_velocity_adjuster {
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
/// Add a circular obstacle at the specified position
void AddCircularObstacle(const double x, const double y, const double radius, const uint32_t point_num,
                         PointCloud& obstacle) {
  // Due to the nature of pcl::cropHull, points or lines without area are filtered out, so make it circular
  for (uint32_t i = 0; i < point_num; ++i) {
    const double theta = (M_PI * 2.0 / static_cast<double>(point_num)) * static_cast<double>(i);
    obstacle.points.push_back(pcl::PointXYZ(x + cos(theta) * radius, y + sin(theta) * radius, 0.0));
  }

  obstacle.height = 1;
  obstacle.width = obstacle.points.size();
  obstacle.is_dense = false;
}

/// Check if the velocity on the 2D plane is identical
bool Is2DVelocitySame(const geometry_msgs::msg::Twist& a, const geometry_msgs::msg::Twist& b) {
  return (std::abs(a.linear.x - b.linear.x) < std::numeric_limits<double>::epsilon() &&
          std::abs(a.linear.y - b.linear.y) < std::numeric_limits<double>::epsilon() &&
          std::abs(a.angular.z - b.angular.z) < std::numeric_limits<double>::epsilon());
}

// Load parameters from a yaml file
void LoadParameterFromYaml(std::shared_ptr<rclcpp::Node> node,
    const std::string& yaml_directory, const std::string& yaml_name) {
  const std::string yaml_path = yaml_directory + yaml_name;
  // Load yaml and generate ParameterMap
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

using WaitFunctionType = std::function<bool()>;
bool WaitUntil(rclcpp::Node::SharedPtr node, WaitFunctionType condition_function,
    double timeout_sec, double rate_hz = 100.0) {
  // Error checking for arguments
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
    if (node != nullptr) rclcpp::spin_some(node);
    if (condition_function()) return true;
    if (rclcpp::Clock(RCL_ROS_TIME).now() >= end_time) break;
    rate.sleep();
  }
  return false;
}
}  // namespace tmc_base_velocity_adjuster
#endif  // TMC_BASE_VELOCITY_ADJUSTER_TEST_UTILS_HPP_
