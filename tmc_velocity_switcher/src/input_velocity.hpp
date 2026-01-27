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

#ifndef TMC_VELOCITY_SWITCHER_INPUT_VELOCITY_HPP_
#define TMC_VELOCITY_SWITCHER_INPUT_VELOCITY_HPP_
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <boost/utility.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace tmc_velocity_switcher {
/// Axis index
enum Axis {
  kAxisX = 0,
  kAxisY = 1,
  kAxisTheta = 2,
  kAxisCnt = 3
};

/// Class to manage input velocity commands
class InputVelocity : private boost::noncopyable {
 public:
  typedef std::shared_ptr<InputVelocity> Ptr;
  // Constructor
  explicit InputVelocity(
      const rclcpp::Node::SharedPtr node, const std::map<std::string, rclcpp::Parameter>& parameters);

  // Returns the currently held Twist value
  geometry_msgs::msg::Twist::Ptr velocity(void) const;

  // Returns the last subscribed time
  double updated_time() const;

  // Get whether the specified axis is under control
  bool as_control_target(const int32_t axis) const;

  // Get velocity priority
  int32_t priority() const;

 private:
  // Subscriber callback
  void Callback(const geometry_msgs::msg::Twist::SharedPtr input_velocity);
  // Subscriber for input velocity commands
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

  // Message for input velocity commands
  geometry_msgs::msg::Twist::Ptr velocity_;
  // Last subscribed time
  double updated_time_;
  // Velocity control target
  std::vector<bool> as_control_target_;
};
}  // namespace tmc_velocity_switcher

#endif  // TMC_VELOCITY_SWITCHER_INPUT_VELOCITY_HPP_
