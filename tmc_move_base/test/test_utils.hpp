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
// Utility functions for testing
#ifndef TMC_MOVE_BASE_TEST_UTILS_HPP_
#define TMC_MOVE_BASE_TEST_UTILS_HPP_

#include <limits>

#include <angles/angles.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// TODO(syuuhei_shiro) 他パッケージ(base_path_planner等)にも同じようなものが存在するため共通化する
namespace tmc_move_base {
const double kPositionErrorThreshold = 0.01;
const double kAngleErrorThreshold = 0.001;

// Check if two points match
bool IsMatchPoseStamped(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2) {
  const double yaw1 = tf2::getYaw(pose1.pose.orientation);
  const double yaw2 = tf2::getYaw(pose2.pose.orientation);
  if (strcmp(pose1.header.frame_id.c_str(), pose2.header.frame_id.c_str()) ||
      fabs(pose1.pose.position.x - pose2.pose.position.x) > kPositionErrorThreshold ||
      fabs(pose1.pose.position.y - pose2.pose.position.y) > kPositionErrorThreshold ||
      fabs(angles::shortest_angular_distance(yaw1, yaw2)) > kAngleErrorThreshold) {
    return false;
  }
  return true;
}

geometry_msgs::msg::Quaternion CreateQuaternionMsgFromYaw(const double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}


// Create the position and orientation of the cart
geometry_msgs::msg::Pose CreatePose(const double x, const double y, const double yaw) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation = CreateQuaternionMsgFromYaw(yaw);
  return pose;
}
}  // namespace tmc_move_base
#endif  // TMC_MOVE_BASE_TEST_UTILS_HPP_
