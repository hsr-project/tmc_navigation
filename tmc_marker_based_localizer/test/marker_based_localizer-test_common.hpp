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

#ifndef TMC_MARKER_BASED_LOCALIZER_MARKER_BASED_LOCALIZER_TEST_COMMON_HPP_
#define TMC_MARKER_BASED_LOCALIZER_MARKER_BASED_LOCALIZER_TEST_COMMON_HPP_

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tmc_marker_based_localizer {
// frame id
constexpr const char* kCameraTfName = "head_l_stereo_camera_frame";
constexpr const char* kDefaultBaseTfName = "base_footprint";
// Response waiting time [s]
constexpr double kNoResultTimeout = 1.0;
// Waiting time [s] for the target node to receive data sent by the test node
constexpr double kReceiveWaitTime = 0.1;
// Cycle [Hz]
constexpr double kRate = 10.0;

// Generate joint_state
sensor_msgs::msg::JointState CreateJointState(const std::vector<std::string>& joints_list);
// Set velocity in joint_state
void SetJointStateVelocity(sensor_msgs::msg::JointState& joint_state,
                           const std::string& joint_name, const double velocity);
// Pose setting
geometry_msgs::msg::Pose CreatePose(const double position_x, const double position_y, const double position_z,
                                    const double orientation_x, const double orientation_y,
                                    const double orientation_z, const double orientation_w);
// Generate pose (tf)
tf2::Transform CreateTransform(const double translation_x, const double translation_y, const double translation_z,
                               const double rotation_x, const double rotation_y,
                               const double rotation_z, const double rotation_w);
// Load parameters from yaml file
void LoadParameterFromYaml(std::shared_ptr<rclcpp::Node> node, const std::string& yaml_directory,
                           const std::string& yaml_name);
}  // namespace tmc_marker_based_localizer

#endif  // TMC_MARKER_BASED_LOCALIZER_MARKER_BASED_LOCALIZER_TEST_COMMON_HPP_
