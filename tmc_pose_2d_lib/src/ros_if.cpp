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
#include "tmc_pose_2d_lib/ros_if.hpp"
#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace tmc_pose_2d_lib {

DistanceMap RosMsg2DistanceMap(const nav_msgs::msg::OccupancyGrid& msg) {
  // Note: The potential_width information is not included in msg, so it needs to be provided separately.

  // Convert the values of OccupancyGrid to the values of DistanceMap.
  // Initialize elements with unknown for the size
  std::vector<unsigned char> data(msg.data.size(), 0);
  for (uint32_t  i = 0; i < msg.data.size(); ++i) {
    // Convert values in [0,100] to values stretched and rounded to [1,255].
    // If negative, it is unknown.
    if (msg.data[i] >= 0) {
      data[i] = static_cast<unsigned char>(round((static_cast<double>(msg.data[i]) / 100.0) * 254.0 + 1.0));
    }
  }

  DistanceMap distance_map(
    Pose2d(msg.info.origin.position.x,
           msg.info.origin.position.y,
           tf2::getYaw(msg.info.origin.orientation)),
    msg.info.resolution, msg.info.width, msg.info.height, data);
  return distance_map;
}
DistanceMap RosMsg2DistanceMap(const tmc_navigation_msgs::msg::OccupancyGridUint& msg) {
  // Note: The potential_width information is not included in msg, so it needs to be provided separately.

  DistanceMap distance_map(
    Pose2d(msg.info.origin.position.x,
           msg.info.origin.position.y,
           tf2::getYaw(msg.info.origin.orientation)),
    msg.info.resolution, msg.info.width, msg.info.height, msg.data);

  return distance_map;
}

Pose2d GetPose2dFromRosMsg(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
  return std::move(
    Pose2d(msg.pose.pose.position.x,
           msg.pose.pose.position.y,
           tf2::getYaw(msg.pose.pose.orientation)));
}

Pose2d GetPose2dFromRosMsg(const geometry_msgs::msg::PoseStamped& msg) {
  return std::move(
    Pose2d(msg.pose.position.x,
           msg.pose.position.y,
           tf2::getYaw(msg.pose.orientation)));
}

Pose2d GetPose2dFromRosMsg(const geometry_msgs::msg::TransformStamped& msg) {
  return std::move(
    Pose2d(msg.transform.translation.x,
           msg.transform.translation.y,
           tf2::getYaw(msg.transform.rotation)));
}

Pose2d GetPose2dFromRosMsg(const geometry_msgs::msg::Pose& msg) {
  return std::move(
    Pose2d(msg.position.x,
           msg.position.y,
           tf2::getYaw(msg.orientation)));
}

geometry_msgs::msg::Pose GetPoseMsg(const Pose2d& pose) {
  tf2::Quaternion quat_tf;
  geometry_msgs::msg::Quaternion quat_msg;
  quat_tf.setRPY(0, 0, pose.theta());
  tf2::convert(quat_tf, quat_msg);
  geometry_msgs::msg::Pose pose_msg;

  pose_msg.position.x = pose.x();
  pose_msg.position.y = pose.y();
  pose_msg.orientation = quat_msg;
  return pose_msg;
}

geometry_msgs::msg::PoseStamped GetPoseStamped(const Pose2d& pose, const double& time, const std::string frame_id) {
  geometry_msgs::msg::PoseStamped msg;
  msg.pose = GetPoseMsg(pose);
  msg.header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9), RCL_ROS_TIME);
  msg.header.frame_id = frame_id;
  return msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped GetPoseWithCovarianceStamped(
  const Pose2d& pose, const double& time, const std::string frame_id) {
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.pose.pose = GetPoseMsg(pose);
  msg.header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9), RCL_ROS_TIME);
  msg.header.frame_id = frame_id;
  return msg;
}

}  // namespace tmc_pose_2d_lib
