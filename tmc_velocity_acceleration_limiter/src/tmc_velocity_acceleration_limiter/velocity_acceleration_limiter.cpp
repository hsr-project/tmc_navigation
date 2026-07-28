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
/// @file velocity_acceleration_limiter.cpp
/// @brief Acceleration and deceleration limits
#include "velocity_acceleration_limiter.hpp"

#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include "param.hpp"

namespace {
// Translational acceleration limit parameter name
constexpr const char* const kLinearAccelerationLimitName = "linear_acceleration_limit";
// Default value for translational acceleration limit [m/s^2]
constexpr double kLinearAccelerationLimitDefault = 1.0;
// Rotational acceleration limit parameter name
constexpr const char* const kAngularAccelerationLimitName = "angular_acceleration_limit";
// Default value for rotational acceleration limit [rad/s^2)]
constexpr double kAngularAccelerationLimitDefault = 1.0;
// Translational deceleration limit parameter name
constexpr const char* const kLinearDecelerationLimitName = "linear_deceleration_limit";
// Default value for translational deceleration limit [m/s^2]
constexpr double kLinearDecelerationLimitDefault = 1.0;
// Rotational deceleration limit parameter name
constexpr const char* const kAngularDecelerationLimitName = "angular_deceleration_limit";
// Default value for rotational deceleration limit [rad/s^2)]
constexpr double kAngularDecelerationLimitDefault = 1.0;
// Velocity input timeout parameter name
constexpr const char* const kInputVelocityTimeoutName = "input_velocity_timeout";
// Default value for velocity input timeout [s)]
constexpr double kInputVelocityTimeoutDefault = 0.2;
// Default enable/disable parameter name for velocity limit
constexpr const char* const kEnableLimitDefaultName = "enable_limit_default";
// Default setting for enabling/disabling velocity limit
constexpr bool kEnableLimitDefaultDefault = true;
// Node drive cycle [Hz]
constexpr double kRate = 100.0;
}  // anonymous namespace

namespace tmc_velocity_acceleration_limiter {
using std::chrono::milliseconds;
using std::placeholders::_1;
using std::placeholders::_2;

/// Constructor
VelocityAccelerationLimiter::VelocityAccelerationLimiter(const rclcpp::NodeOptions& options)
    : Node("velocity_acceleration_limiter", options),
      command_velocity_(Eigen::Vector3d::Zero()),
      last_velocity_(Eigen::Vector3d::Zero()),
      last_velocity_subscribed_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
      last_velocity_limited_time_(rclcpp::Clock(RCL_ROS_TIME).now()) {}

/// Initialization
void VelocityAccelerationLimiter::Init() {
  sub_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "command_velocity", 1, std::bind(&VelocityAccelerationLimiter::VelocityCallback, this, _1));
  pub_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>("acceleration_limited_velocity", 1);

  srv_enable_ = this->create_service<std_srvs::srv::Empty>("~/enable",
      std::bind(&VelocityAccelerationLimiter::EnableServiceCallback, this, _1, _2));

  srv_disable_ = this->create_service<std_srvs::srv::Empty>("~/disable",
      std::bind(&VelocityAccelerationLimiter::DisableServiceCallback, this, _1, _2));
  // Retrieve acceleration limit parameters
  const double linear_acceleration_limit =
      GetPositiveDoubleParam(shared_from_this(), kLinearAccelerationLimitName, kLinearAccelerationLimitDefault);
  const double angular_acceleration_limit =
      GetPositiveDoubleParam(shared_from_this(), kAngularAccelerationLimitName, kAngularAccelerationLimitDefault);
  acceleration_limit_ << linear_acceleration_limit, linear_acceleration_limit, angular_acceleration_limit;

  // Retrieve deceleration limit parameters
  const double linear_deceleration_limit =
      GetPositiveDoubleParam(shared_from_this(), kLinearDecelerationLimitName, kLinearDecelerationLimitDefault);
  const double angular_deceleration_limit =
      GetPositiveDoubleParam(shared_from_this(), kAngularDecelerationLimitName, kAngularDecelerationLimitDefault);
  deceleration_limit_ << linear_deceleration_limit, linear_deceleration_limit, angular_deceleration_limit;

  // Retrieve timeout parameters
  input_velocity_timeout_ =
      GetPositiveDoubleParam(shared_from_this(), kInputVelocityTimeoutName, kInputVelocityTimeoutDefault);

  // Retrieve default parameters for enabling/disabling limit functionality
  GetOptionalParam(shared_from_this(), kEnableLimitDefaultName, is_enable_limit_, kEnableLimitDefaultDefault);

  // Set main processing execution timer
  node_action_timer_ = this->create_wall_timer(milliseconds(static_cast<int32_t>(1000 / kRate)),
      std::bind(&VelocityAccelerationLimiter::NodeActionTimerCallback, this));
}

/// Core processing for speed limitation
void VelocityAccelerationLimiter::NodeActionTimerCallback() {
  // Calculate time interval since last speed limit processing
  const rclcpp::Time current_time = rclcpp::Clock(RCL_ROS_TIME).now();
  const double interval = (current_time - last_velocity_limited_time_).seconds();
  last_velocity_limited_time_ = current_time;
  // Issue speed if input speed is not interrupted or previous speed is not zero
  const double elapsed_from_last_subscribed =
      (rclcpp::Clock(RCL_ROS_TIME).now() - last_velocity_subscribed_time_).seconds();
  if ((elapsed_from_last_subscribed <= input_velocity_timeout_) ||
      (last_velocity_.norm() > std::numeric_limits<double>::epsilon())) {
    // Set command speed to zero if speed input is interrupted
    if (elapsed_from_last_subscribed > input_velocity_timeout_) {
      command_velocity_ = Eigen::Vector3d::Zero();
    }
    Eigen::Vector3d limit_velocity = command_velocity_;
    if (is_enable_limit_) {
      for (int32_t i = 0; i < limit_velocity.size(); ++i) {
        // Determine acceleration/deceleration based on the magnitude of previous and input speeds, and switch limits
        double limit = 0.0;
        if (std::abs(limit_velocity(i)) >= std::abs(last_velocity_(i))) {
          limit = acceleration_limit_(i);
        } else {
          limit = deceleration_limit_(i);
        }
        // Apply speed limitation if the speed difference exceeds the product of acceleration/deceleration limit and time interval
        const double dv = limit_velocity(i) - last_velocity_(i);
        if (std::abs(dv) - std::abs(limit * interval) > std::numeric_limits<double>::epsilon()) {
          const double sign = dv / std::abs(dv);
          limit_velocity(i) = last_velocity_(i) + sign * limit * interval;
        }
      }
    }
    // Issue speed
    geometry_msgs::msg::Twist output_velocity;
    output_velocity.linear.x = limit_velocity(0);
    output_velocity.linear.y = limit_velocity(1);
    output_velocity.angular.z = limit_velocity(2);
    pub_velocity_->publish(output_velocity);
    last_velocity_ = limit_velocity;
  }
}


/// Input speed callback
void VelocityAccelerationLimiter::VelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  command_velocity_ << msg->linear.x, msg->linear.y, msg->angular.z;
  last_velocity_subscribed_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
}


/// Enable functionality service
void VelocityAccelerationLimiter::EnableServiceCallback(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res) {
  is_enable_limit_ = true;
}

/// Disable functionality service
void VelocityAccelerationLimiter::DisableServiceCallback(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res) {
  is_enable_limit_ = false;
}
}  // namespace tmc_velocity_acceleration_limiter

