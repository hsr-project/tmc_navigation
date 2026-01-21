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
/// @file move_base.cpp
/// @brief Autonomous movement action node
#include "tmc_move_base/move_base.hpp"

#include <string>
#include <vector>

// TODO(syuuhei_shiro): LoggerをROS2化する
// #include <ros/file_log.h>

namespace {
/// Autonomous movement action name
const char* const kMoveBaseGoalActionName = "move_base/move";
/// Autonomous movement action goal name
const char* const kMoveBaseGoalActionGoalName = "move_base/move/goal";
/// Autonomous movement goal topic name
const char* const kGoalTopicName = "move_base_simple/goal";
/// Path planning action name
const char* const kPathPlannerActionName = "base_path_plan";
/// Self-position topic name
const char* const kGlobalPoseTopicName = "global_pose";
/// Topic buffer size
const uint32_t kTopicBufferSize = 10;
/// Action cycle [Hz]
const double kRate = 10.0;
/// TF waiting time [s]
const double kWaitTFDuration = 2.0;
/// Path planning action completion wait timeout
const double kWaitResultTimeout = 5.0;

/// Timeout parameter name for path planning failure state
const char* const kPlanningTimeoutParameterName = "planning_timeout";
/// Global coordinate frame parameter name
const char* const kGlobalFrameParameterName = "floor_frame";

/// Default timeout for path planning failure state [s]
const double kDefaultPlanningTimeOut = 10.0;
/// Default name for global coordinate frame
const char* const kDefaultGlobalFrameName = "map";

// TODO(syuuhei_shiro): パラメータ取得関数は共通パッケージに置く
// Retrieve required parameters
template <typename T>
bool GetParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value) {
  rclcpp::Parameter param;
  if (!node->get_parameter(param_name, param)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("move_base"),
        "Parameter '" << param_name << "' is not specified.");
    return false;
  }
  value = param.get_value<T>();
  return true;
}

// Retrieve optional parameters
template <typename T>
void GetOptionalParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value,
                      const T& default_value) {
  if (!GetParam(node, param_name, value)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("move_base"),
        "Used default value: " << default_value);
    value = default_value;
  }
}
}  // anonymous namespace


namespace tmc_move_base {
using std::placeholders::_1;
using std::placeholders::_2;
// Convert feedback from path planning action to text
std::string PathPlannerStatusToString(const PathPlanAction::Feedback& status) {
  if (status.status == PathPlanAction::Feedback::RUNNING) {
    return "RUNNING";
  } else {
    uint32_t reason = status.reason;
    switch (reason) {
      case PathPlanAction::Feedback::NONE:
        return "PLANNING, reason: NONE";
      case PathPlanAction::Feedback::PATH_PLANNING_FAIL:
        return "PLANNING, reason: PATH_PLANNING_FAIL";
      case PathPlanAction::Feedback::ROBOT_IS_ON_STATIC_OBSTACLE:
        return "PLANNING, reason: ROBOT_IS_ON_STATIC_OBSTACLE";
      case PathPlanAction::Feedback::ROBOT_IS_ON_DYNAMIC_OBSTACLE:
        return "PLANNING, reason: ROBOT_IS_ON_DYNAMIC_OBSTACLE";
      case PathPlanAction::Feedback::GOAL_IS_ON_DYNAMIC_OBSTACLE:
        return "PLANNING, reason: GOAL_IS_ON_DYNAMIC_OBSTACLE";
    }
  }
  return std::string();
}

/// Coordinate transformation
bool TransformPoseStamped(const tf2_ros::Buffer& tf_buffer, const geometry_msgs::msg::PoseStamped& in_pose,
                          const std::string& frame_id, geometry_msgs::msg::PoseStamped& out_pose) {
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer.lookupTransform(
        frame_id, in_pose.header.frame_id, in_pose.header.stamp, rclcpp::Duration::from_seconds(kWaitTFDuration));
    tf2::doTransform(in_pose, out_pose, transform_stamped);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("move_base"), "Couldn't transform \'%s\' to \'%s\': %s",
                 in_pose.header.frame_id.c_str(), frame_id.c_str(), ex.what());
    return false;
  }
  return true;
}

/// Constructor
MoveBase::MoveBase(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("move_base", options),
      rate_(kRate),
      planner_goal_handle_(nullptr),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {}

// Initialization
void MoveBase::Init() {
  // Subscriber setup
  global_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      kGlobalPoseTopicName, kTopicBufferSize, std::bind(&MoveBase::GlobalPoseCallback, this, _1));
  goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      kGoalTopicName, kTopicBufferSize, std::bind(&MoveBase::GoalTopicCallback, this, _1));

  GetOptionalParam(shared_from_this(), kPlanningTimeoutParameterName, planning_timeout_, kDefaultPlanningTimeOut);
  if (planning_timeout_ <= 0.0) {
    RCLCPP_WARN_STREAM(this->get_logger(), kPlanningTimeoutParameterName << " is invalid. Use default value.");
    planning_timeout_ = kDefaultPlanningTimeOut;
  }
  GetOptionalParam(shared_from_this(), kGlobalFrameParameterName,
                   global_frame_name_, std::string(kDefaultGlobalFrameName));

  // Create path planning client
  path_plan_action_client_ = rclcpp_action::create_client<PathPlanAction>(this, kPathPlannerActionName);
  /// Create action client for own action server
  move_base_action_client_ = rclcpp_action::create_client<MoveBaseAction>(this, kMoveBaseGoalActionName);

  // Launch action server
  action_server_ = std::make_shared<SimpleActionServer<MoveBaseAction>>(
      shared_from_this(),
      kMoveBaseGoalActionName,
      std::bind(&MoveBase::MoveBaseActionCallback, this));
  action_server_->activate();
}

void MoveBase::MoveBaseActionCallback() {
  const auto goal = action_server_->get_current_goal();
  auto result = std::make_shared<MoveBaseAction::Result>();
  // Convert goal to global coordinate system
  geometry_msgs::msg::PoseStamped global_goal;
  if (!TransformPoseStamped(tf_buffer_, goal->pose, global_frame_name_, global_goal)) {
    action_server_->terminate_current(result);
    return;
  }
  // Send path planning action
  SendPathPlanAction(global_goal);
  rclcpp::Time last_running_time = rclcpp::Clock(RCL_ROS_TIME).now();
  while (rclcpp::ok()) {
    // Copy feedback from path planning action locally
    PathPlanAction::Feedback current_status = planner_feedback_;

    if (action_server_->is_preempt_requested()) {
      // If a new goal is received, terminate without stopping path planning
      action_server_->terminate_current(result);
      return;
    }
    // Cancel check
    if (action_server_->is_cancel_requested() || !action_server_->is_server_active()) {
      RCLCPP_DEBUG(this->get_logger(), "ActionMoveBase PREEMPTED.");
      // If canceled, stop path planning and terminate
      CancelPathPlanAction();
      action_server_->terminate_current(result);
      return;
    }

    // Check the state of the path planning action
    auto state = planner_goal_handle_->get_status();
    if (state == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
        state == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
      if (current_status.status == PathPlanAction::Feedback::RUNNING) {
        last_running_time = rclcpp::Clock(RCL_ROS_TIME).now();
      } else {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Base path was not made.");
        if ((rclcpp::Clock(RCL_ROS_TIME).now() - last_running_time).seconds() > planning_timeout_) {
          // If a state other than RUNNING persists and times out, stop path planning and terminate with ABORTED
          RCLCPP_ERROR_STREAM(this->get_logger(),
              "Path plan timeout.(" << PathPlannerStatusToString(current_status) << ")");
          CancelPathPlanAction();
          // TODO(syuuhei_shiro): LoggerをROS2化する
          // logger_.RecordLog();
          action_server_->terminate_current(result);
          return;
        }
      }
    } else if (state == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
      RCLCPP_DEBUG(this->get_logger(), "Arrived Move Base goal. ActionMoveBase SUCCEEDED.");
      action_server_->succeeded_current(result);
      planner_goal_handle_ = nullptr;
      return;
    } else if (state == rclcpp_action::GoalStatus::STATUS_CANCELED ||
        state == rclcpp_action::GoalStatus::STATUS_CANCELING) {
      RCLCPP_INFO(this->get_logger(), "%s action was canceled.", kPathPlannerActionName);
      action_server_->terminate_current(result);
      planner_goal_handle_ = nullptr;
      return;
    } else {
      RCLCPP_ERROR(this->get_logger(), "planner aborted with result code : %d", planner_result_.value().result->reason);
      // TODO(syuuhei_shiro): LoggerをROS2化する
      // logger_.RecordLog();
      action_server_->terminate_current(result);
      planner_goal_handle_ = nullptr;
      return;
    }
    // Publish current cart position as feedback
    auto feedback = std::make_shared<MoveBaseAction::Feedback>();
    feedback->current_pose = current_global_pose_;
    action_server_->publish_feedback(feedback);
    rate_.sleep();
  }
}

void MoveBase::goal_response_callback(const PathPlanGoalHandle::SharedPtr& future) {
  // TODO(syuuhei_shiro): Goalがサーバからrejectされた場合のチェック
}

void MoveBase::feedback_callback(PathPlanGoalHandle::SharedPtr,
    const std::shared_ptr<const PathPlanAction::Feedback> feedback) {
  planner_feedback_ = *feedback;
}

void MoveBase::result_callback(const PathPlanGoalHandle::WrappedResult& result) {
  planner_result_ = result;
}

/// Self-position callback
/// TODO(syuuhei_shiro) 自己位置はPoseWithCoverianceStamped型で受け、尤度低下に対する異常系を実装する
void MoveBase::GlobalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  current_global_pose_ = *msg;
}

/// Autonomous movement goal topic callback
void MoveBase::GoalTopicCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // Convert to own action and send
  auto move_base_action_goal = MoveBaseAction::Goal();
  move_base_action_goal.pose = *msg;
  move_base_action_client_->async_send_goal(move_base_action_goal);
}


/// Send path planning action request
void MoveBase::SendPathPlanAction(const geometry_msgs::msg::PoseStamped& goal_pose) {
  auto send_goal_options = rclcpp_action::Client<PathPlanAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&MoveBase::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&MoveBase::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&MoveBase::result_callback, this, _1);


  auto planner_action_goal = PathPlanAction::Goal();
  planner_action_goal.goal = goal_pose;
  auto goal_handle_future = path_plan_action_client_->async_send_goal(planner_action_goal, send_goal_options);
  planner_goal_handle_ = goal_handle_future.get();
  planner_result_ = std::nullopt;
}

/// Cancel path planning action
void MoveBase::CancelPathPlanAction() {
  if (planner_goal_handle_) {
    // Cancel path following
    path_plan_action_client_->async_cancel_goal(planner_goal_handle_);
    planner_goal_handle_ = nullptr;
  }
}

}  // namespace tmc_move_base
