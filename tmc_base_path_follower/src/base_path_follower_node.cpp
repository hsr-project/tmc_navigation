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
/// @file base_path_follower_node.cpp
/// @brief Omnidirectional cart path following node
#include <tmc_base_path_follower/base_path_follower_node.hpp>

#include <limits>
#include <optional>
#include <vector>

#include <tmc_base_path_follower/base_path_follower_factory.hpp>
#include <tmc_base_path_follower/parameter_creator.hpp>
#include <tmc_pose_2d_lib/ros_if.hpp>


using tmc_pose_2d_lib::GetPose2dFromRosMsg;

namespace {
// Topic buffer size
constexpr uint32_t kTopicBufferSize = 10;
// ACTION server connection wait time
constexpr double kTimeWaitForActionServer = 1.0;
// Node drive cycle
constexpr double kFrequency = 100.0;
/// Default value for self-position timeout
constexpr double kGlobalPoseTimeoutDefault = 1.0;
}  // anonymous namespace

namespace tmc_base_path_follower {
using std::placeholders::_1;

/// Convert nav_msgs::msg::Path type to PoseSeq
PoseSeq ConvertPathToPoseSeq(const nav_msgs::msg::Path& path) {
  PoseSeq pose_seq;
  for (uint32_t i = 0; i < path.poses.size(); ++i) {
    geometry_msgs::msg::Pose pose = path.poses[i].pose;
    const Pose2d pose2d = GetPose2dFromRosMsg(pose);
    pose_seq.push_back(pose2d);
  }
  return pose_seq;
}

// Constructor
BasePathFollowerNode::BasePathFollowerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("base_path_follower", options), rate_(kFrequency), last_velocity_(Vector3d::Zero()) {
}

void BasePathFollowerNode::Init() {
  // Parameter reading
  LoadParameter();

  // Autonomous movement command velocity publisher
  velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("base_velocity", kTopicBufferSize);

  // Self-position subscriber
  global_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "global_pose", kTopicBufferSize, std::bind(&BasePathFollowerNode::GlobalPoseCallback, this, _1));

  // Path subscriber
  path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
      "~/path", 1, std::bind(&BasePathFollowerNode::PathCallback, this, _1));

  // Path generation function
  path_info_creator_.reset(new PathInfoCreator(CreatePathInfoCreatorParameter(shared_from_this())));
  // Path following function
  base_path_follower_ = CreateBasePathFollower(shared_from_this());

  action_server_ = std::make_shared<SimpleActionServer<PathFollowActionServer>>(
      shared_from_this(),
      "path_follow_action",
      std::bind(&BasePathFollowerNode::PathFollowerActionCallBack, this));
  action_server_->activate();

  action_client_ = rclcpp_action::create_client<PathFollowActionServer>(this, "path_follow_action");
}


// Self-position callback
void BasePathFollowerNode::GlobalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  current_global_pose_ = *msg;
}

void BasePathFollowerNode::PathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  PathFollowActionServer::Goal action_goal;
  action_goal.path = *msg;
  action_client_->async_send_goal(action_goal);
}


void BasePathFollowerNode::PathFollowerActionCallBack() {
  bool first_time = true;
  auto goal = action_server_->get_current_goal();
  auto action_result = std::make_shared<PathFollowActionServer::Result>();
  // Following path
  PoseSeq origin_path;
  PathInfo path_info;
  while (rclcpp::ok()) {
    if (action_server_->is_preempt_requested() || first_time) {
      if (!first_time) {
        RCLCPP_INFO(this->get_logger(), "Path follow action server is preempt requested.");
        goal = action_server_->accept_pending_goal();
      }
      first_time = false;
      // If a path with less than 2 points is input, publish velocity 0 and cancel action
      if (goal->path.poses.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "The number of Path points is less than 2. Abort follow action.");
        const Vector3d velocity = Vector3d::Zero();
        PublishVelocity(velocity);
        action_server_->terminate_current(action_result);
        return;
      }
      // Following path generation
      origin_path = ConvertPathToPoseSeq(goal->path);
      path_info = path_info_creator_->CreatePathInfo(origin_path);
      // Path following control class initialization
      base_path_follower_->Initialize(path_info);
      rate_.sleep();
      continue;
    }

    if (action_server_->is_cancel_requested()) {
      RCLCPP_INFO(this->get_logger(), "Path follow action server is requested cancel.");
      // If canceled, publish velocity 0 and exit
      PublishVelocity(Vector3d::Zero());
      action_server_->terminate_current(action_result);
      return;
    }

    // Self-position timeout check
    if (!CheckGlobalPoseTimeout()) {
      // If self-position is timed out, publish velocity 0 and exit
      RCLCPP_ERROR(this->get_logger(), "Global pose timeout. Abort follow action.");
      PublishVelocity(Vector3d::Zero());
      action_server_->terminate_current(action_result);
      return;
    }

    const Pose2d global_pose = GetPose2dFromRosMsg(current_global_pose_);
    bool is_arrived_goal = false;
    uint32_t current_path_index;
    Vector3d velocity;
    // Path following control velocity
    const bool result = base_path_follower_->FollowPathVelocity(
        global_pose, last_velocity_, 1.0 / kFrequency,
        is_arrived_goal, current_path_index, velocity);
    if (!result) {
      RCLCPP_ERROR(this->get_logger(), "FollowPathVelocity failed. Abort follow action.");
      PublishVelocity(Vector3d::Zero());
      action_server_->terminate_current(action_result);
      return;
    }
    FeedbackProgress(path_info, current_path_index);
    // If arrived at the goal, publish completion of following and exit
    if (is_arrived_goal) {
      // Publish velocity 0 and exit normally
      RCLCPP_INFO(this->get_logger(), "Arrived goal. Stop base.");
      velocity = Vector3d::Zero();
      PublishVelocity(velocity);
      action_server_->succeeded_current(action_result);
      return;
    }
    // Velocity issuance
    PublishVelocity(velocity);
    rate_.sleep();
  }
}

/// Feedback on path following action progress rate
void BasePathFollowerNode::FeedbackProgress(const PathInfo& path_info, const uint32_t current_path_index) {
  const double path_length = path_info.splined_path_left_lengths.at(0);
  // Fail-safe check to prevent out-of-range access and division by zero
  if (path_info.splined_path_left_lengths.size() > 0 && path_length > std::numeric_limits<double>::epsilon() &&
      current_path_index >= 0 && current_path_index < path_info.splined_path_left_lengths.size()) {
    double progress = 0.0;
    // Calculation of arrival rate to the goal
    progress = 1.0 - (path_info.splined_path_left_lengths.at(current_path_index) / path_length);
    auto feedback = std::make_shared<PathFollowActionServer::Feedback>();
    feedback->progress = progress;
    action_server_->publish_feedback(feedback);
  }
}

/// Issuance of Velocity
void BasePathFollowerNode::PublishVelocity(const Vector3d& velocity) {
  last_velocity_ = velocity;
  geometry_msgs::msg::Twist command_velocity;
  command_velocity.linear.x = velocity(kPoseX);
  command_velocity.linear.y = velocity(kPoseY);
  command_velocity.angular.z = velocity(kPoseTheta);
  velocity_publisher_->publish(command_velocity);
}


// Parameter reading
void BasePathFollowerNode::LoadParameter() {
  GetOptionalParam(shared_from_this(), "global_pose_timeout", global_pose_timeout_, kGlobalPoseTimeoutDefault);
  if (global_pose_timeout_ < std::numeric_limits<double>::epsilon()) {
    RCLCPP_WARN(this->get_logger(), "Parameter global_pose_timeout is not set or invalid. Use default value.");
    global_pose_timeout_ = kGlobalPoseTimeoutDefault;
  }
}

// Self-position timeout check
bool BasePathFollowerNode::CheckGlobalPoseTimeout() {
  rclcpp::Time now = this->get_clock()->now();
  rclcpp::Time stamp = rclcpp::Time(current_global_pose_.header.stamp);
  return (now - stamp).seconds() < global_pose_timeout_;
}

}  // namespace tmc_base_path_follower
