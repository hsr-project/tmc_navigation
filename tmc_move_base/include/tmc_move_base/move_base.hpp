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
/// @file move_base.hpp
/// @brief Autonomous Movement Action Node
#ifndef TMC_MOVE_BASE_MOVE_BASE_HPP_
#define TMC_MOVE_BASE_MOVE_BASE_HPP_
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_util/simple_action_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tmc_move_base/logger.hpp>
#include <tmc_navigation_msgs/action/base_path_plan.hpp>

namespace tmc_move_base {
using MoveBaseAction = nav2_msgs::action::NavigateToPose;
using MoveBaseGoalHandle = rclcpp_action::ServerGoalHandle<MoveBaseAction>;
using PathPlanAction = tmc_navigation_msgs::action::BasePathPlan;
using PathPlanGoalHandle = rclcpp_action::ClientGoalHandle<PathPlanAction>;
using nav2_util::SimpleActionServer;

/// Autonomous Movement Action Class
class MoveBase : public rclcpp::Node {
 public:
  explicit MoveBase(const rclcpp::NodeOptions& options);
  ~MoveBase() {}
  // Initialization
  void Init();

 private:
  // Autonomous Movement Action Callback
  void MoveBaseActionCallback();

  // Self-Position Callback
  void GlobalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  // Autonomous Movement Goal Topic Callback
  void GoalTopicCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // // Path Planning Action Client Callback
  void goal_response_callback(const PathPlanGoalHandle::SharedPtr& future);
  void feedback_callback(PathPlanGoalHandle::SharedPtr,
      const std::shared_ptr<const PathPlanAction::Feedback> feedback);
  void result_callback(const PathPlanGoalHandle::WrappedResult& result);

  // Send Path Planning Action Request
  void SendPathPlanAction(const geometry_msgs::msg::PoseStamped& goal_pose);
  // Cancel Path Planning Action
  void CancelPathPlanAction();

  // Node Drive Cycle
  rclcpp::Rate rate_;

  // Action Server
  std::shared_ptr<SimpleActionServer<MoveBaseAction>> action_server_;
  /// Path Planning Action Client
  rclcpp_action::Client<PathPlanAction>::SharedPtr path_plan_action_client_;
  /// Path Planning Goal Handle
  PathPlanGoalHandle::SharedPtr planner_goal_handle_;
  // Action Client for Own Action Server
  rclcpp_action::Client<MoveBaseAction>::SharedPtr move_base_action_client_;
  // Feedback from Path Planning Action
  PathPlanAction::Feedback planner_feedback_;
  // Result of Path Planning Action
  std::optional<PathPlanGoalHandle::WrappedResult> planner_result_;

  // Autonomous Movement Goal Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
  // Self-Position Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_subscriber_;

  // tf Buffer
  tf2_ros::Buffer tf_buffer_;
  // tf Listener
  tf2_ros::TransformListener tf_listener_;
  // Self-Position
  geometry_msgs::msg::PoseStamped current_global_pose_;
  // Global Coordinate Frame Name
  std::string global_frame_name_;
  // Timeout for Path Planning Failure State [s]
  double planning_timeout_;
  // TODO(syuuhei_shiro): LoggerをROS2化する
  // Logger
  // Logger logger_;
};

}  // namespace tmc_move_base

#endif  // TMC_MOVE_BASE_MOVE_BASE_HPP_
