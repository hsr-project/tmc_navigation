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
#ifndef TMC_MOVE_BASE_PATH_PLANNER_DUMMY_HPP_
#define TMC_MOVE_BASE_PATH_PLANNER_DUMMY_HPP_
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_util/simple_action_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tmc_navigation_msgs/action/base_path_plan.hpp>

namespace tmc_move_base {
using nav2_util::SimpleActionServer;
using PathPlanAction = tmc_navigation_msgs::action::BasePathPlan;

/// Dummy class for PathPlanner
class PathPlannerDummy : public rclcpp::Node {
 public:
  explicit PathPlannerDummy(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("planner_dummy_node", options), is_requested_(false), is_canceled_(false), is_running_(false),
      action_complete_time_(3.0), action_result_(rclcpp_action::ResultCode::SUCCEEDED) {}

  void Init(const double rate) {
    rate_ = std::make_shared<rclcpp::Rate>(rate);
    action_feedback_ = std::make_shared<PathPlanAction::Feedback>();
    action_feedback_->status = PathPlanAction::Feedback::RUNNING;
    action_feedback_->reason = PathPlanAction::Feedback::NONE;
    dummy_server_ = std::make_shared<SimpleActionServer<PathPlanAction>>(
        shared_from_this(),
        "base_path_plan",
        std::bind(&PathPlannerDummy::PathPlannerActionCB, this));
    dummy_server_->activate();
  }

  // Set the termination condition for the action
  void SetActionCompleteCondition(const double action_complete_time,
                                  const rclcpp_action::ResultCode& action_result) {
    action_complete_time_ = action_complete_time;
    action_result_ = action_result;
  }

  // Set the feedback for the action
  void SetActionFeedback(const PathPlanAction::Feedback& action_feedback) {
    *action_feedback_ = action_feedback;
  }

  // Whether a path planning action has been requested
  bool IsRequested() {
    const bool ret = is_requested_;
    // Return to false after checking
    is_requested_ = false;
    return ret;
  }

  // Whether a path planning action has been canceled
  bool IsCanceled() {
    const bool ret = is_canceled_;
    // Return to false after checking
    is_canceled_ = false;
    return ret;
  }

  // Whether a path planning action is in progress
  bool IsRunning() {
    return is_running_;
  }

  // Get the currently requested goal
  geometry_msgs::msg::PoseStamped current_requested_goal() const {
    return current_requested_goal_;
  }

  void Run() {
    killed_ = false;
    while (rclcpp::ok() && !killed_) {
      rate_->sleep();
      rclcpp::spin_some(shared_from_this());
    }
  }
  void Kill() {
    killed_ = true;
  }

  // Stub for path planning action
  void PathPlannerActionCB() {
    current_requested_goal_ = dummy_server_->get_current_goal()->goal;
    is_running_ = true;
    is_requested_ = true;
    rclcpp::Time time = rclcpp::Clock(RCL_ROS_TIME).now();
    auto action_result = std::make_shared<PathPlanAction::Result>();
    while (rclcpp::Clock(RCL_ROS_TIME).now() - time < rclcpp::Duration::from_seconds(action_complete_time_)) {
      rate_->sleep();
      if (dummy_server_->is_preempt_requested()) {
        dummy_server_->terminate_current(action_result);
        is_running_ = false;
        return;
      }
      if (dummy_server_->is_cancel_requested()) {
        dummy_server_->terminate_current(action_result);
        is_canceled_ = true;
        is_running_ = false;
        return;
      }
      dummy_server_->publish_feedback(action_feedback_);
    }
    if (action_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
      dummy_server_->succeeded_current(action_result);
    } else {
      dummy_server_->terminate_current(action_result);
    }
    is_running_ = false;
    return;
  }

 private:
  std::shared_ptr<SimpleActionServer<PathPlanAction>> dummy_server_;
  // Action cycle
  std::shared_ptr<rclcpp::Rate> rate_;
  // Time until action completion
  double action_complete_time_;
  // Feedback of the action
  std::shared_ptr<PathPlanAction::Feedback> action_feedback_;
  // Result of the action
  rclcpp_action::ResultCode action_result_;
  // Currently requested goal
  geometry_msgs::msg::PoseStamped current_requested_goal_;
  // Whether a request has occurred
  bool is_requested_;
  // Whether a cancellation has occurred
  bool is_canceled_;
  // Whether the action is in progress
  bool is_running_;
  // Flag to stop the running Run
  bool killed_;
};
}  // namespace tmc_move_base
#endif  // TMC_MOVE_BASE_PATH_PLANNER_DUMMY_HPP_
