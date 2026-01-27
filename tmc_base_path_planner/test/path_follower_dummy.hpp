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
#include <memory>

#include <nav2_util/simple_action_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tmc_navigation_msgs/action/path_follower.hpp>

namespace tmc_base_path_planner {
using nav2_util::SimpleActionServer;
using PathFollowActionServer = tmc_navigation_msgs::action::PathFollower;

/// Dummy class for PathFollower
class PathFollowerDummy : public rclcpp::Node {
 public:
  explicit PathFollowerDummy(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("follower_dummy_node", options),
      set_action_comlete_time_(5.0), set_action_result_(rclcpp_action::ResultCode::SUCCEEDED),
      is_requested_(false), is_canceled_(false), killed_(false) {}

  void Init(const double rate) {
    rate_ = std::make_shared<rclcpp::Rate>(rate);
    dummy_server_ = std::make_shared<SimpleActionServer<PathFollowActionServer>>(
        shared_from_this(),
        "path_follow_action",
        std::bind(&PathFollowerDummy::PathFollowerActionCB, this));
    dummy_server_->activate();
  }

  // Set completion conditions for path following action
  void SetActionCompleteCondition(const double action_comlete_time,
                                  const rclcpp_action::ResultCode& action_result) {
    set_action_comlete_time_ = action_comlete_time;
    set_action_result_ = action_result;
  }

  // Whether the path following action was requested
  bool IsRequested() {
    const bool ret = is_requested_;
    // Return to false after checking
    is_requested_ = false;
    return ret;
  }


  // Whether the path following action was canceled
  bool IsCanceled() {
    const bool ret = is_canceled_;
    // Return to false after checking
    is_canceled_ = false;
    return ret;
  }

  // Get the currently requested path
  nav_msgs::msg::Path CurrentRequestedPath() const {
    return current_requested_path_;
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

 private:
  // Stub for path following action
  void PathFollowerActionCB() {
    auto goal = dummy_server_->get_current_goal();
    current_requested_path_ = goal->path;
    is_requested_ = true;
    rclcpp::Time start = this->get_clock()->now();
    auto action_result = std::make_shared<PathFollowActionServer::Result>();
    while (this->get_clock()->now() - start < rclcpp::Duration::from_seconds(set_action_comlete_time_)) {
      rate_->sleep();
      if (dummy_server_->is_preempt_requested()) {
        is_requested_ = true;
        goal = dummy_server_->accept_pending_goal();
        current_requested_path_ = goal->path;
      }
      if (dummy_server_->is_cancel_requested()) {
        dummy_server_->terminate_current(action_result);
        is_canceled_ = true;
        return;
      }
    }
    if (set_action_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
      dummy_server_->succeeded_current(action_result);
    } else {
      dummy_server_->terminate_current(action_result);
    }
    return;
  }

  std::shared_ptr<SimpleActionServer<PathFollowActionServer>> dummy_server_;
  std::shared_ptr<rclcpp::Rate> rate_;
  // Time until action completion
  double set_action_comlete_time_;
  // Result of the action
  rclcpp_action::ResultCode set_action_result_;
  // Currently requested path
  nav_msgs::msg::Path current_requested_path_;
  // Whether a request occurred
  bool is_requested_;
  // Whether a cancellation occurred
  bool is_canceled_;
  // Flag to stop the running Run
  bool killed_;
};
}  // namespace tmc_base_path_planner
