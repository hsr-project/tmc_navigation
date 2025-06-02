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
#ifndef TMC_POSE_2D_LIB_ROS_IF_HPP_
#define TMC_POSE_2D_LIB_ROS_IF_HPP_

#include <string>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tmc_navigation_msgs/msg/occupancy_grid_uint.hpp>
#include "tmc_pose_2d_lib/distance_map.hpp"
#include "tmc_pose_2d_lib/pose_2d.hpp"

namespace tmc_pose_2d_lib {
DistanceMap RosMsg2DistanceMap(const tmc_navigation_msgs::msg::OccupancyGridUint& msg);
DistanceMap RosMsg2DistanceMap(const nav_msgs::msg::OccupancyGrid& msg);

Pose2d GetPose2dFromRosMsg(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
Pose2d GetPose2dFromRosMsg(const geometry_msgs::msg::TransformStamped& msg);
Pose2d GetPose2dFromRosMsg(const geometry_msgs::msg::Pose& msg);
Pose2d GetPose2dFromRosMsg(const geometry_msgs::msg::PoseStamped& msg);

geometry_msgs::msg::Pose GetPoseMsg(const Pose2d& pose);
geometry_msgs::msg::PoseStamped GetPoseStamped(const Pose2d& pose, const double& time, const std::string frame_id);
geometry_msgs::msg::PoseWithCovarianceStamped GetPoseWithCovarianceStamped(
  const Pose2d& pose, const double& time, const std::string frame_id);

}  // namespace tmc_pose_2d_lib

#endif  // TMC_POSE_2D_LIB_ROS_IF_HPP_
