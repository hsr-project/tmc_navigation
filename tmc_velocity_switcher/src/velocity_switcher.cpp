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
/// select input velocity of higher priority then output it smoothly.
/// Copyright (C) 2019 TOYOTA Motor Corporation.
#include "velocity_switcher.hpp"
#include <chrono>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "param.hpp"

namespace {
// node name
const char* const kNodeName = "velocity_switcher";
// output topic name
const char* const kTopicNameOutputVelocity = "output_velocity";
// parameter server name
const char* const kParamNameVelocityTimeout = "timeout";            // Timeout duration for input velocity [sec]
const char* const kParamNameSwichingPeriod = "switching_period";    // Switching duration when changing input velocity [sec]
const char* const kParamNameInputVelocities = "input_velocities";   // Input velocity parameter group
const char* const kParamNamePriority = "priority";                  // Input velocity priority; lower values indicate higher priority
// parameter default values
const double kTimeoutInputVelocity = 1.00;  // Default timeout duration for input velocity commands [sec]
const double kSwitchingPeriod = 0.50;       // Default switching duration for input velocity commands [sec]
// operation cycle [Hz]
const double kControlCycle = 200.0;
}  // anonymous namespace

namespace tmc_velocity_switcher {
using std::chrono::milliseconds;
// constructor
VelocitySwitcher::VelocitySwitcher(const rclcpp::NodeOptions& options)
    : Node("velocity_switcher", options), has_input_velocity_(false) {}

void VelocitySwitcher::Init() {
  // retrieve ros parameters
  UpdateParameters();
  // register publisher
  pub_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>(kTopicNameOutputVelocity, 100);
  // set main processing execution timer
  node_action_timer_ = this->create_wall_timer(milliseconds(static_cast<int32_t>(1000 / kControlCycle)),
      std::bind(&VelocitySwitcher::NodeActionTimerCallback, this));
}

// update velocity multiplier for each axis
void VelocitySwitcher::UpdateAxisRatio(const uint32_t axis) {
  double now = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
  int32_t current_priority = std::numeric_limits<int32_t>::max();
  uint32_t high_priority_index = std::numeric_limits<uint32_t>::max();
  // select the highest priority valid velocity input
  for (uint32_t i = 0; i < input_velocities_.size(); ++i) {
    int32_t priority = input_velocities_[i].first;
    if (current_priority > priority &&
        input_velocities_[i].second->as_control_target(axis) &&
        input_velocities_[i].second->velocity() &&
        (now - input_velocities_[i].second->updated_time() < velocity_timeout_)) {
      current_priority = priority;
      high_priority_index = i;
    }
  }
  // update weights
  for (uint32_t i = 0; i < input_velocities_.size(); ++i) {
    if (i == high_priority_index) {
      /// slightly increase the weight of the highest priority valid velocity input
      /// the increase amount transitions from 0 to 1 over switching_period_ seconds
      ratio_[axis][i] += 1.0 / (switching_period_ * kControlCycle);

      if (ratio_[axis][i] > 1.0) {
        // cap the weight at 1 if it exceeds 1
        ratio_[axis][i] = 1.0;
      }
    } else {
      /// decrease the weight of all other valid velocity inputs
      /// the decrease amount transitions from 1 to 0 over switching_period_ seconds
      if (input_velocities_[i].second->as_control_target(axis)) {
        ratio_[axis][i] -= 1.0 / (switching_period_ * kControlCycle);
        if (ratio_[axis][i] < 0.0) {
          // cap the weight at 0 if it falls below 0
          ratio_[axis][i] = 0.0;
        }
      }
    }
  }
}

// update velocity multiplier
void VelocitySwitcher::UpdateRatio() {
  UpdateAxisRatio(kAxisX);
  UpdateAxisRatio(kAxisY);
  UpdateAxisRatio(kAxisTheta);
}

// publish output velocity
void VelocitySwitcher::OutputVelocity() {
  std::vector<double> velocity(kAxisCnt);
  std::vector<double> weight(kAxisCnt);

  for (uint32_t i = 0; i < input_velocities_.size(); ++i) {
    if (!input_velocities_[i].second->velocity()) {
      continue;
    }
    if (input_velocities_[i].second->as_control_target(kAxisX)) {
      velocity[kAxisX] += input_velocities_[i].second->velocity()->linear.x * ratio_[kAxisX][i];
      weight[kAxisX] += ratio_[kAxisX][i];
    }
    if (input_velocities_[i].second->as_control_target(kAxisY)) {
      velocity[kAxisY] += input_velocities_[i].second->velocity()->linear.y * ratio_[kAxisY][i];
      weight[kAxisY] += ratio_[kAxisY][i];
    }
    if (input_velocities_[i].second->as_control_target(kAxisTheta)) {
      velocity[kAxisTheta] += input_velocities_[i].second->velocity()->angular.z * ratio_[kAxisTheta][i];
      weight[kAxisTheta] += ratio_[kAxisTheta][i];
    }
  }

  /// if there is no input velocity (all weights are 0), do not output velocity
  /// only output velocity on the first occurrence of transitioning from having input to no input
  if (weight[kAxisX] == 0.0 && weight[kAxisY] == 0.0 && weight[kAxisTheta] == 0.0) {
    if (!has_input_velocity_) {
      return;
    }
    has_input_velocity_ = false;
  } else {
    has_input_velocity_ = true;
  }

  geometry_msgs::msg::Twist output_velocity;
  // control x
  if (weight[kAxisX] < 0.001) {
    // to prevent division by zero, set velocity to 0 when weights are small
    output_velocity.linear.x = 0.0;
  } else if (weight[kAxisX] > 1.0) {
    // normalize weights to 1 when they exceed 1
    output_velocity.linear.x = velocity[kAxisX] / weight[kAxisX];
  } else {
    // output weights as-is when they are between 0 and 1
    output_velocity.linear.x = velocity[kAxisX];
  }

  // control y
  if (weight[kAxisY] < 0.001) {
    // to prevent division by zero, set velocity to 0 when weights are small
    output_velocity.linear.y = 0.0;
  } else if (weight[kAxisY] > 1.0) {
    // normalize weights to 1 when they exceed 1
    output_velocity.linear.y = velocity[kAxisY] / weight[kAxisY];
  } else {
    // output weights as-is when they are between 0 and 1
    output_velocity.linear.y = velocity[kAxisY];
  }

  // control t
  if (weight[kAxisTheta] < 0.001) {
    // to prevent division by zero, set velocity to 0 when weights are small
    output_velocity.angular.z = 0.0;
  } else if (weight[kAxisTheta] > 1.0) {
    // normalize weights to 1 when they exceed 1
    output_velocity.angular.z = velocity[kAxisTheta] / weight[kAxisTheta];
  } else {
    // output weights as-is when they are between 0 and 1
    output_velocity.angular.z = velocity[kAxisTheta];
  }
  pub_velocity_->publish(output_velocity);
}

// retrieve parameters
void VelocitySwitcher::UpdateParameters() {
  GetOptionalParam(shared_from_this(), kParamNameVelocityTimeout, velocity_timeout_, kTimeoutInputVelocity);
  if (velocity_timeout_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "velocity_timeout parameter value [%lf] is invalid. Use default value [%lf].",
        velocity_timeout_, kTimeoutInputVelocity);
    velocity_timeout_ = kTimeoutInputVelocity;
  }

  GetOptionalParam(shared_from_this(), kParamNameSwichingPeriod, switching_period_, kSwitchingPeriod);
  if (switching_period_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "switching_period parameter value [%lf] is invalid. Use default value [%lf].",
        switching_period_, kSwitchingPeriod);
    switching_period_ = kSwitchingPeriod;
  }

  std::map<std::string, rclcpp::Parameter> input_velocities;
  GetGroupParam(shared_from_this(), kParamNameInputVelocities, input_velocities);
  std::vector<std::string> name_list;
  for (auto it = input_velocities.begin(); it != input_velocities.end(); ++it) {
    const int32_t sbstr_index = it->first.find(".");
    if (sbstr_index == std::string::npos) {
      RCLCPP_INFO(this->get_logger(), "%s is invalid", it->first.c_str());
      continue;
    }
    const std::string input_velocity_name = it->first.substr(0, sbstr_index);

    if (std::find(name_list.begin(), name_list.end(), input_velocity_name) == name_list.end()) {
      name_list.push_back(input_velocity_name);
      std::map<std::string, rclcpp::Parameter> input_velocity_param;
      GetGroupParam(input_velocities, input_velocity_name, input_velocity_param);
      int32_t priority;
      if (!GetParam(input_velocity_param, kParamNamePriority, priority)) {
        throw std::runtime_error("priority parameter was not found.");
      }
      std::pair<int32_t, InputVelocity::Ptr> input_velocity =
          std::make_pair(priority, InputVelocity::Ptr(new InputVelocity(shared_from_this(), input_velocity_param)));
      input_velocities_.push_back(input_velocity);
    }
  }

  ratio_ = std::vector<std::vector<double> >(kAxisCnt, std::vector<double>(input_velocities_.size(), 0.0));
}

// callback function for main processing execution timer
void VelocitySwitcher::NodeActionTimerCallback() {
  UpdateRatio();
  OutputVelocity();
}
}  // namespace tmc_velocity_switcher
