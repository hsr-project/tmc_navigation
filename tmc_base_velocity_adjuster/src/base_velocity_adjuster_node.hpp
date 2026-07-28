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
/// @file base_velocity_adjuster_node.hpp
/// @brief Cart speed input correction function node
#ifndef TMC_BASE_VELOCITY_ADJUSTER_BASE_VELOCITY_ADJUSTER_NODE_HPP_
#define TMC_BASE_VELOCITY_ADJUSTER_BASE_VELOCITY_ADJUSTER_NODE_HPP_
#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>

#include "base_velocity_optimizer.hpp"
#include "obstacle_input.hpp"
#include "param.hpp"

namespace tmc_base_velocity_adjuster {
using std::placeholders::_1;
using std::placeholders::_2;

/// @brief Cart speed input correction class
template<class RosMsg, class Data>
class BaseVelocityAdjusterNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit BaseVelocityAdjusterNode(const rclcpp::NodeOptions& options)
      : Node("base_velocity_adjuster", options) {}

  /// Initialization
  void Init() {
    // Subscriber
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "command_velocity", 1, std::bind(&BaseVelocityAdjusterNode::VelocityCallback, this, _1));
    // Publisher
    pub_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>("adjusted_velocity", 1);
    // Create speed optimization class
    velocity_optimizer_.reset(new BaseVelocityOptimizer<Data>(shared_from_this()));
    obstacle_input_.reset(new ObstacleInput<RosMsg, Data>(shared_from_this()));


    // Get the default state of correction function enable/disable
    GetOptionalParam(shared_from_this(), "enable_adjustment_default", is_enable_adjustment_, true);
    // Service
    srv_enable_ = this->create_service<std_srvs::srv::Empty>("~/enable",
        std::bind(&BaseVelocityAdjusterNode::EnableServiceCallback, this, _1, _2));
    srv_disable_ = this->create_service<std_srvs::srv::Empty>("~/disable",
        std::bind(&BaseVelocityAdjusterNode::DisableServiceCallback, this, _1, _2));
  }

 private:
  /// @brief Speed input callback Corrects the input speed to the optimal speed and publishes
  /// @param [in] msg Input speed topic
  void VelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (is_enable_adjustment_) {
      const auto input_velocity = Eigen::Vector3d(msg->linear.x, msg->linear.y, msg->angular.z);

      // Use obstacle input information to correct the input speed to the optimal speed
      auto adjusted_velocity = input_velocity;
      velocity_optimizer_->OptimizeVelocity(input_velocity, obstacle_input_->GetObstacle(), adjusted_velocity);
      geometry_msgs::msg::Twist output_velocity;
      output_velocity.linear.x = adjusted_velocity(kX);
      output_velocity.linear.y = adjusted_velocity(kY);
      output_velocity.angular.z = adjusted_velocity(kYaw);
      pub_velocity_->publish(output_velocity);
    } else {
      pub_velocity_->publish(*msg);
    }
  }

  /// @brief Enable function service
  void EnableServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res) {
    is_enable_adjustment_ = true;
  }

  /// @brief Disable function service
  void DisableServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res) {
    is_enable_adjustment_ = false;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_velocity_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_enable_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_disable_;
  // Enable/disable speed adjustment
  bool is_enable_adjustment_;

  // Obstacle input
  std::unique_ptr<ObstacleInput<RosMsg, Data>> obstacle_input_;
  // Speed optimization
  std::unique_ptr<BaseVelocityOptimizer<Data>> velocity_optimizer_;
};

}  // namespace tmc_base_velocity_adjuster

#endif  // TMC_BASE_VELOCITY_ADJUSTER_BASE_VELOCITY_ADJUSTER_NODE_HPP_
