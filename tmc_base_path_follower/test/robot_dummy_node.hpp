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
#ifndef TMC_BASE_PATH_FOLLOWER_ROBOT_DUMMY_NODE_HPP_
#define TMC_BASE_PATH_FOLLOWER_ROBOT_DUMMY_NODE_HPP_

#include <memory>

#include <angles/angles.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tmc_base_path_follower {
// Test drive cycle [s]
constexpr double kCycleTime = 0.01;
// Timeout duration for velocity command input [s]
constexpr double kTwistTimeOut = 0.5;

using std::placeholders::_1;
using std::placeholders::_2;
/// Dummy node class for the robot
/// Receives velocity and simulates robot movement
/// Performs the following operations
/// - When receiving an initial position movement command from the test node, it continuously moves to that position at a constant speed
/// - After reaching the initial position, it moves according to the velocity from base_path_follower
class RobotDummyNode : public rclcpp::Node {
 public:
  explicit RobotDummyNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("robot_dummy_node", options),
      last_twist_subscribed_time_(this->get_clock()->now()),
      moving_to_initial_pose_(false),
      publish_global_pose_(true) {}

  void Init() {
    current_pose_.pose.position.x = 0.0;
    current_pose_.pose.position.y = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    current_pose_.pose.orientation = tf2::toMsg(q);

    // Test subscriber/publisher settings
    pub_global_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/global_pose", 1);

    sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/initial_pose", 1,
        std::bind(&RobotDummyNode::InitialPoseCallback, this, _1));

    sub_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>("/base_velocity", 1,
        std::bind(&RobotDummyNode::VelocityCallback, this, _1));
    // Start/stop service for publishing self-position
    start_publish_pose_service_ = this->create_service<std_srvs::srv::Empty>("/start_publish_pose",
        std::bind(&RobotDummyNode::StartPublishPoseServiceCallback, this, _1, _2));
    stop_publish_pose_service_ = this->create_service<std_srvs::srv::Empty>("/stop_publish_pose",
        std::bind(&RobotDummyNode::StopPublishPoseServiceCallback, this, _1, _2));
    // This node operates passively and does not wait for communication establishment
  }

  /// Main processing
  void Run() {
    killed_ = false;
    rclcpp::Rate loop_rate(1.0 / kCycleTime);
    while (rclcpp::ok() && !killed_) {
      // Moves to the initial position during the initial position movement command from the test node, otherwise moves according to velocity
      if (moving_to_initial_pose_) {
        current_pose_ = initial_pose_;
        moving_to_initial_pose_ = false;
      } else {
        UpdateCurrentPose();
      }
      if (publish_global_pose_) {
        // Self-position publishing
        current_pose_.header.stamp = this->get_clock()->now();
        pub_global_pose_->publish(current_pose_);
      }
      loop_rate.sleep();
      rclcpp::spin_some(shared_from_this());
    }
  }

  void Kill() {
    killed_ = true;
  }

 private:
  /// Cart velocity command callback
  void VelocityCallback(const geometry_msgs::msg::Twist::SharedPtr velocity) {
    velocity_ = *velocity;
    last_twist_subscribed_time_ = this->get_clock()->now();
  }

  /// Initial position specification callback from the test node
  void InitialPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    moving_to_initial_pose_ = true;
    initial_pose_ = *pose;
  }

  /// Start service for publishing self-position
  void StartPublishPoseServiceCallback(
      std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res) {
    publish_global_pose_ = true;
  }

  /// Stop service for publishing self-position
  void StopPublishPoseServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
                                      std_srvs::srv::Empty::Response::SharedPtr res) {
    publish_global_pose_ = false;
  }

  /// Update self-position
  void UpdateCurrentPose() {
    // If no velocity command is received for a certain period, the velocity is set to 0
    if (this->get_clock()->now() - last_twist_subscribed_time_ >
        rclcpp::Duration::from_seconds(kTwistTimeOut)) {
      velocity_.linear.x = 0.0;
      velocity_.linear.y = 0.0;
      velocity_.angular.z = 0.0;
    }
    // Self-position calculation
    // Since the cart's velocity is based on the front direction of the cart, it is converted to absolute coordinates for calculation
    double yaw = tf2::getYaw(current_pose_.pose.orientation);
    current_pose_.pose.position.x += (velocity_.linear.x * cos(yaw) - velocity_.linear.y * sin(yaw)) * kCycleTime;
    current_pose_.pose.position.y += (velocity_.linear.x * sin(yaw) + velocity_.linear.y * cos(yaw)) * kCycleTime;
    yaw += velocity_.angular.z * kCycleTime;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angles::normalize_angle(yaw));
    current_pose_.pose.orientation = tf2::toMsg(q);
  }

  // Self-position publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_global_pose_;
  // Cart velocity command subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_velocity_;
  // Initial position subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_initial_pose_;
  // Cart velocity command value
  geometry_msgs::msg::Twist velocity_;
  // Self-position
  geometry_msgs::msg::PoseStamped current_pose_;
  // Initial position
  geometry_msgs::msg::PoseStamped initial_pose_;
  // Moving to the initial position
  bool moving_to_initial_pose_;
  // Time of the last subscribed velocity command
  rclcpp::Time last_twist_subscribed_time_;
  // Start service for publishing self-position
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_publish_pose_service_;
  // Stop service for publishing self-position
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_publish_pose_service_;
  // Whether to publish self-position
  bool publish_global_pose_;
  // Stop flag
  bool killed_;
};

}  // namespace tmc_base_path_follower
#endif  // TMC_BASE_PATH_FOLLOWER_ROBOT_DUMMY_NODE_HPP_
