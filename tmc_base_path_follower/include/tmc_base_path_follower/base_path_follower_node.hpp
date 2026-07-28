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
/// @file base_path_follower_node.hpp
/// @brief Cart path following node
#ifndef TMC_BASE_PATH_FOLLOWER_BASE_PATH_FOLLOWER_NODE_HPP_
#define TMC_BASE_PATH_FOLLOWER_BASE_PATH_FOLLOWER_NODE_HPP_
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_util/simple_action_server.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tmc_navigation_msgs/action/path_follower.hpp>

#include "base_path_follower.hpp"
#include "common.hpp"
#include "path_info_creator.hpp"

namespace tmc_base_path_follower {
using nav2_util::SimpleActionServer;
// Omnidirectional cart following control node class
class BasePathFollowerNode : public rclcpp::Node {
 public:
  // Constructor
  explicit BasePathFollowerNode(const rclcpp::NodeOptions& options);
  // Destructor
  ~BasePathFollowerNode() {}
  // Initialization
  void Init();

 private:
  using PathFollowActionServer = tmc_navigation_msgs::action::PathFollower;
  using PathFollowGoalHandle = rclcpp_action::ServerGoalHandle<PathFollowActionServer>;

  // Path following ACTION callback
  void PathFollowerActionCallBack();
  // Estimated self-position acquisition callback
  void GlobalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  // Path acquisition callback
  void PathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  // Feedback on path following action progress rate
  void FeedbackProgress(const PathInfo& path_info, const uint32_t current_path_index);
  // Velocity publishing
  void PublishVelocity(const Vector3d& velocity);
  // Parameter reading
  void LoadParameter();
  // Self-position timeout check
  bool CheckGlobalPoseTimeout();

  /// Node cycle time
  rclcpp::Rate rate_;

  /// Path following ACTION server
  std::shared_ptr<SimpleActionServer<PathFollowActionServer>> action_server_;

  /// Self-position subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_subscriber_;
  /// Path subscriber
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
  /// Cart command velocity publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  /// Path following action goal publisher
  /// Subscribe to path and send action goal to itself
  rclcpp_action::Client<PathFollowActionServer>::SharedPtr action_client_;
  /// Path information generation function
  PathInfoCreator::Ptr path_info_creator_;
  /// Path following class
  BasePathFollower::Ptr base_path_follower_;
  // Current self-position
  geometry_msgs::msg::PoseStamped current_global_pose_;
  // Self-position timeout duration
  double global_pose_timeout_;
  // Command velocity of the previous step
  Vector3d last_velocity_;
};

}  // namespace tmc_base_path_follower
#endif /*TMC_BASE_PATH_FOLLOWER_BASE_PATH_FOLLOWER_NODE_HPP_*/
