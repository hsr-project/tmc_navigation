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
/// @file viewpoint_to_tracking_target.cpp
/// @brief Calculate the angle to direct the viewpoint towards the target object
#include <tmc_viewpoint_controller/viewpoint_to_tracking_target.hpp>

#include <string>
#include <angles/angles.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_viewpoint_controller/param.hpp>

namespace {
/// Path topic name
const char* const kTargetPathTopicName = "target_path";
}  // end anonymous namespace

namespace tmc_viewpoint_controller {
using std::placeholders::_1;
/// Constructor
ViewpointToTrackingTarget::ViewpointToTrackingTarget(const rclcpp::Node::SharedPtr node) {
  // Target trajectory Subscriber setup
  target_path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      kTargetPathTopicName, 1, std::bind(&ViewpointToTrackingTarget::TargetPathCallback, this, _1));
}

/// Destructor
ViewpointToTrackingTarget::~ViewpointToTrackingTarget() {}

/// Target direction viewpoint calculation
bool ViewpointToTrackingTarget::ViewpointToTrackingTargetDircetion(
    const Eigen::Vector3d& robot_pose, double& out_direction) {
  if (!target_path_.poses.empty()) {
    // If the target can be tracked, direct the viewpoint to the latest position of the target
    double x = target_path_.poses.back().pose.position.x - robot_pose(0);
    double y = target_path_.poses.back().pose.position.y - robot_pose(1);
    out_direction = angles::normalize_angle(atan2(y, x) - robot_pose(2));
    return true;
  }
  return false;
}

/// Target trajectory callback
void ViewpointToTrackingTarget::TargetPathCallback(const nav_msgs::msg::Path::SharedPtr target_path) {
  target_path_ = *target_path;
}
}  // end namespace tmc_viewpoint_controller
