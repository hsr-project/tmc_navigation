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
#ifndef TMC_ODOMETRY_SWITCHER_ODOMETRY_SWITCHER_HPP_
#define TMC_ODOMETRY_SWITCHER_ODOMETRY_SWITCHER_HPP_

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tmc_navigation_msgs/srv/odometry_switch.hpp>


namespace tmc_odometry_switcher {
using OdometryList = std::map<std::string, Eigen::Affine3d, std::less<std::string>,
                     Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d> > >;
/// Odometry Switching Class
/// Ensure that the output odometry does not jump before and after switching.
/// Calculate the odometry by adding the differences from the position during the odometry switch as a reference.
class OdometrySwitcher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<OdometrySwitcher>;
  /// Constructor
  /// @param init_odom_type [I] Type of odometry to use initially
  /// @param odom_list [I] Odometry list
  OdometrySwitcher(const std::string& init_odom_type, const OdometryList& odom_list)
      : source_odom_type_(init_odom_type),
        odom_list_(odom_list),
        base_transform_(Eigen::Affine3d::Identity()) {}

  // Initialization
  void Init();

  /// Odometry Switch
  /// @param odom_type [I] Type of odometry
  /// @ret true: switch successful / false: switch failed
  bool SwitchSourceOdom(const std::string& odom_type);

  /// Odometry Update
  /// @param odom_type [I] Type of odometry
  /// @param odom_pose [I] Odometry position
  /// @param output_odom_pose [O] Updated odometry position
  /// @ret true: odometry updated / false: odometry not updated
  bool UpdateOdometry(const std::string& odom_type, const Eigen::Affine3d& odom_pose,
                      Eigen::Affine3d& output_odom_pose);

 private:
  /// Calculate Odometry Position
  Eigen::Affine3d GetCurrentOdometryTransform() const;

  // Type of source odometry to be used
  std::string source_odom_type_;
  // Odometry list
  OdometryList odom_list_;
  // Reference transformation during odometry switching
  Eigen::Affine3d base_transform_;
};


class OdometryPublisher {
 public:
  using Ptr = std::shared_ptr<OdometryPublisher>;
  OdometryPublisher(rclcpp::Node::SharedPtr node,
      const std::string& odom_frame, const std::string& odom_child_frame);
  void PublishOdometry(const nav_msgs::msg::Odometry& odometry);

 private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::string odom_frame_;
  std::string odom_child_frame_;
};


class OdometrySubscriber {
 public:
  using Ptr = std::shared_ptr<OdometrySubscriber>;
  OdometrySubscriber(rclcpp::Node::SharedPtr node,
      const std::string& odom_type, const std::string& topic_name,
      const OdometrySwitcher::Ptr& switcher, const OdometryPublisher::Ptr& publisher);
 private:
  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
  std::string odom_type_;

  OdometrySwitcher::Ptr odometry_switcher_;
  OdometryPublisher::Ptr odometry_publisher_;
};


class OdometrySwitcherNode : public rclcpp::Node {
 public:
  explicit OdometrySwitcherNode(const rclcpp::NodeOptions& options);
  void Init();

 private:
  // Odometry switch service callback
  bool SwitchServiceCallback(
      tmc_navigation_msgs::srv::OdometrySwitch::Request::SharedPtr req,
      tmc_navigation_msgs::srv::OdometrySwitch::Response::SharedPtr res);

  std::vector<OdometrySubscriber::Ptr> odom_subscribers_;
  OdometrySwitcher::Ptr odometry_switcher_;
  rclcpp::Service<tmc_navigation_msgs::srv::OdometrySwitch>::SharedPtr switch_server_;
};
}  // namespace tmc_odometry_switcher

#endif /*TMC_ODOMETRY_SWITCHER_ODOMETRY_SWITCHER_HPP_*/
