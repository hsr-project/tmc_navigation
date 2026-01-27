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
/// select input velocity of higher priority then output it smoothly.
/// Copyright (C) 2019 TOYOTA Motor Corporation.
#include "input_velocity.hpp"
#include <string>
#include <vector>
#include <geometry_msgs/msg/twist.hpp>

#include "param.hpp"

namespace {
// parameter server name
// input speed Topic name
const char* const kParamNameTopicName = "topic_name";
// Whether to control the X-axis direction of input speed
const char* const kParamNameAsControlTargetX = "as_control_target_x";
// Whether to control the Y-axis direction of input speed
const char* const kParamNameAsControlTargetY = "as_control_target_y";
// Whether to control the T-axis direction of input speed
const char* const kParamNameAsControlTargetT = "as_control_target_theta";

// parameter default value
const bool kAsControlTarget = true;  // Default value for whether to control
}  // anonymous namespace

namespace tmc_velocity_switcher {
using std::placeholders::_1;
// Constructor
InputVelocity::InputVelocity(
    const rclcpp::Node::SharedPtr node, const std::map<std::string, rclcpp::Parameter>& parameters)
    : updated_time_(0) {
  std::string topic_name;
  if (!GetParam(parameters, kParamNameTopicName, topic_name)) {
    throw std::runtime_error("topic_name parameter was not found.");
  }
  sub_ = node->create_subscription<geometry_msgs::msg::Twist>(topic_name, 100,
      std::bind(&InputVelocity::Callback, this, _1));
  as_control_target_.resize(kAxisCnt);
  bool as_control_target;
  GetOptionalParam(parameters, kParamNameAsControlTargetX, as_control_target, kAsControlTarget);
  as_control_target_[kAxisX] = as_control_target;
  GetOptionalParam(parameters, kParamNameAsControlTargetY, as_control_target, kAsControlTarget);
  as_control_target_[kAxisY] = as_control_target;
  GetOptionalParam(parameters, kParamNameAsControlTargetT, as_control_target, kAsControlTarget);
  as_control_target_[kAxisTheta] = as_control_target;
}

// Get whether the specified axis is a control target
bool InputVelocity::as_control_target(const int32_t axis) const {
  return as_control_target_[axis];
}

// Subscriber callback
void InputVelocity::Callback(const geometry_msgs::msg::Twist::SharedPtr input_velocity) {
  updated_time_ = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
  velocity_ = input_velocity;
}

// Return the currently held Twist value
geometry_msgs::msg::Twist::Ptr InputVelocity::velocity() const {
  return velocity_;
}

// Return the last subscribed time
double InputVelocity::updated_time() const {
  return updated_time_;
}
}  // namespace tmc_velocity_switcher
