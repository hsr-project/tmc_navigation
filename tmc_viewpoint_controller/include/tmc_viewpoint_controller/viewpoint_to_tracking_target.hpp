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
/// @file viewpoint_to_tracking_target.hpp
/// @brief Calculate the angle to direct the viewpoint towards the target object

#ifndef TMC_VIEWPOINT_CONTROLLER_VIEWPOINT_TO_TRACKING_TARGET_HPP_
#define TMC_VIEWPOINT_CONTROLLER_VIEWPOINT_TO_TRACKING_TARGET_HPP_
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace tmc_viewpoint_controller {

class ViewpointToTrackingTarget {
 public:
  typedef std::shared_ptr<ViewpointToTrackingTarget> Ptr;
  explicit ViewpointToTrackingTarget(const rclcpp::Node::SharedPtr node);
  virtual ~ViewpointToTrackingTarget();
  /// Viewpoint calculation
  bool ViewpointToTrackingTargetDircetion(const Eigen::Vector3d& robot_pose, double& out_direction);

 private:
  /// Target trajectory callback
  void TargetPathCallback(const nav_msgs::msg::Path::SharedPtr target_path);

  /// Target trajectory subscriber
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_path_sub_;
  /// Target trajectory
  nav_msgs::msg::Path target_path_;
};
}  // end namespace tmc_viewpoint_controller

#endif  // TMC_VIEWPOINT_CONTROLLER_VIEWPOINT_TO_TRACKING_TARGET_HPP_
