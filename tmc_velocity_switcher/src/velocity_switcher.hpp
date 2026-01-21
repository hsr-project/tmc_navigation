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

#ifndef TMC_VELOCITY_SWITCHER_VELOCITY_SWITCHER_HPP_
#define TMC_VELOCITY_SWITCHER_VELOCITY_SWITCHER_HPP_
#include <map>
#include <utility>
#include <vector>
#include <boost/utility.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include "input_velocity.hpp"

namespace tmc_velocity_switcher {

/// A class that manages multiple velocity inputs (InputVelocity) and switches velocities
class VelocitySwitcher : public rclcpp::Node, private boost::noncopyable {
 public:
  // Constructor
  explicit VelocitySwitcher(const rclcpp::NodeOptions& options);

  void Init();

  // Callback function for the main processing execution timer
  void NodeActionTimerCallback();

 private:
  // Publish output velocity
  void OutputVelocity();
  // Update velocity multiplier
  void UpdateRatio();
  // Update velocity multiplier for each axis
  void UpdateAxisRatio(const uint32_t axis);
  // Retrieve parameters
  void UpdateParameters();
  // Timer for main processing execution
  rclcpp::TimerBase::SharedPtr node_action_timer_;
  // Publisher for output velocity
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity_;
  // Vector that manages the priority and objects of multiple InputVelocities
  std::vector<std::pair<int32_t, InputVelocity::Ptr> > input_velocities_;
  // Weight of each velocity input command
  std::vector<std::vector<double> > ratio_;
  // Timeout period until invalidation when input velocity command is interrupted
  double velocity_timeout_;  // [sec]
  /// Time required to switch to a new velocity command
  /// During that period, the output velocity command changes smoothly
  double switching_period_;  // [sec]
  /// Presence of input velocity Yes:true / No:false
  bool has_input_velocity_;
};
}  // namespace tmc_velocity_switcher

#endif  // TMC_VELOCITY_SWITCHER_VELOCITY_SWITCHER_HPP_
