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

#include "marker_based_localizer-test_common.hpp"

#include <rclcpp/parameter_map.hpp>

namespace tmc_marker_based_localizer {
// Generate joint_state
sensor_msgs::msg::JointState CreateJointState(const std::vector<std::string>& joints_list) {
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  joint_state.name.resize(joints_list.size());
  joint_state.position.resize(joints_list.size());
  joint_state.velocity.resize(joints_list.size());
  joint_state.effort.resize(joints_list.size());
  for (size_t i = 0; i < joints_list.size(); i++) {
    joint_state.name[i] = joints_list[i];
  }
  return joint_state;
}

// Set velocity in joint_state
void SetJointStateVelocity(sensor_msgs::msg::JointState& joint_state,
                           const std::string& joint_name, const double velocity) {
  for (size_t i = 0; i < joint_state.name.size(); i++) {
    if (joint_state.name[i] == joint_name) {
      joint_state.velocity[i] = velocity;
      return;
    }
  }
}

// Set pose
geometry_msgs::msg::Pose CreatePose(const double position_x, const double position_y, const double position_z,
                                    const double orientation_x, const double orientation_y,
                                    const double orientation_z, const double orientation_w) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = position_x;
  pose.position.y = position_y;
  pose.position.z = position_z;
  pose.orientation.x = orientation_x;
  pose.orientation.y = orientation_y;
  pose.orientation.z = orientation_z;
  pose.orientation.w = orientation_w;
  return pose;
}

// Generate pose (tf)
tf2::Transform CreateTransform(const double translation_x, const double translation_y, const double translation_z,
                               const double rotation_x, const double rotation_y,
                               const double rotation_z, const double rotation_w) {
  tf2::Transform tf_pose;
  const geometry_msgs::msg::Pose pose = CreatePose(translation_x, translation_y, translation_z,
                                                   rotation_x, rotation_y, rotation_z, rotation_w);
  tf2::convert(pose, tf_pose);
  return tf_pose;
}

// Load parameters from yaml file
void LoadParameterFromYaml(std::shared_ptr<rclcpp::Node> node, const std::string& yaml_directory,
                           const std::string& yaml_name) {
  const std::string yaml_path = yaml_directory + yaml_name;
  // Load yaml and generate ParameterMap
  const rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_params_t* yaml_params = rcl_yaml_node_struct_init(allocator);
  rcl_parse_yaml_file(yaml_path.c_str(), yaml_params);
  const rclcpp::ParameterMap yaml_param_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);
  // Set ros parameters to node
  const std::string parameter_space = "/" + std::string(node->get_name());
  auto iter = yaml_param_map.find(parameter_space);
  for (auto& param : iter->second) {
    node->declare_parameter(param.get_name(), param.get_type());
    node->set_parameter(param);
  }
}
}  // namespace tmc_marker_based_localizer
