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
/// @file velocity_acceleration_limiter.hpp
/// @brief Acceleration and deceleration limit for speed
#ifndef TMC_VELOCITY_ACCELERATION_LIMITER_VELOCITY_ACCELERATION_LIMITER_HPP_
#define TMC_VELOCITY_ACCELERATION_LIMITER_VELOCITY_ACCELERATION_LIMITER_HPP_
#include <Eigen/Core>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

namespace tmc_velocity_acceleration_limiter {

/// Speed acceleration and deceleration limit class
class VelocityAccelerationLimiter : public rclcpp::Node {
 public:
  explicit VelocityAccelerationLimiter(const rclcpp::NodeOptions& options);
  ~VelocityAccelerationLimiter() = default;
  void Init();

 private:
  // Callback function for main processing execution timer
  void NodeActionTimerCallback();
  /// Input speed callback
  void VelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  /// Enable functionality service
  void EnableServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);
  /// Disable functionality service
  void DisableServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);

  // Timer for main processing execution
  rclcpp::TimerBase::SharedPtr node_action_timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_velocity_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_enable_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_disable_;
  // Enable/disable acceleration and deceleration limit functionality
  bool is_enable_limit_;
  // Command speed
  Eigen::Vector3d command_velocity_;
  // Previously calculated speed
  Eigen::Vector3d last_velocity_;
  // Acceleration and deceleration limit
  Eigen::Vector3d acceleration_limit_;
  Eigen::Vector3d deceleration_limit_;
  // Time of last speed input
  rclcpp::Time last_velocity_subscribed_time_;
  // Time of last acceleration and deceleration limit calculation
  rclcpp::Time last_velocity_limited_time_;
  // Timeout duration [s] to determine speed input interruption
  double input_velocity_timeout_;
};
}  // namespace tmc_velocity_acceleration_limiter

#endif /*TMC_VELOCITY_ACCELERATION_LIMITER_VELOCITY_ACCELERATION_LIMITER_HPP_*/
