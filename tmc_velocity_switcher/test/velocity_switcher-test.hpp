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
/// Header file for testing the velocity_switcher node
/// Copyright (C) 2023 TOYOTA Motor Corporation.

#ifndef TMC_VELOCITY_SWITCHER_VELOCITY_SWITCHER_TEST_HPP_
#define TMC_VELOCITY_SWITCHER_VELOCITY_SWITCHER_TEST_HPP_

#include <memory>
#include <string>
#include <utility>
#include <geometry_msgs/msg/twist.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "../src/velocity_switcher.hpp"

namespace tmc_velocity_switcher {

class TestNode : public rclcpp::Node {
 public:
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // Initialization
  void Init();

  // Check for timeout, returns true if timed out
  bool CheckTimeOut(const rclcpp::Time& start_time) const;

  // Wait for the publisher link to be established
  bool WaitForPublishersLinked();
  // Wait for the subscriber link to be established
  bool WaitForSubscriberLinked();

  // Wait for the subscriber to receive the topic
  bool WaitForSubscriberReceived();

  // Returns true if there is a change in the subscribed data from the previous value
  bool CheckSubscribedDataChange();

  // Callback function
  void CallbackOutput(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Wait until the value of output_velocity settles to 0.0
  void WaitUntilOutputGetsZero();

  // Publish input velocity 0
  void PublishVelocity0();
  // Publish input velocity 1
  void PublishVelocity1();
  // Publish input velocity 2
  void PublishVelocity2();
  // Publish input velocity 3
  void PublishVelocity3();
  // Publish input velocity 4
  void PublishVelocity4();
  // Publish input velocity 5
  void PublishVelocity5();
  // Publish input velocity 6
  void PublishVelocity6();

  // spin
  void SpinOnce();

  // Clear the reception flag
  void ClearSubscribedFlag() { subscribed_flag_ = false; }
  geometry_msgs::msg::Twist output_velocity() const { return output_velocity_; }

 private:
  virtual void SetTestParam();

  // Initial reception flag for the subscriber
  bool subscribed_flag_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity0_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity1_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity2_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity3_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity4_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity5_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity6_pub_;
  // Input message
  geometry_msgs::msg::Twist input_velocity0_;
  geometry_msgs::msg::Twist input_velocity1_;
  geometry_msgs::msg::Twist input_velocity2_;
  geometry_msgs::msg::Twist input_velocity3_;
  geometry_msgs::msg::Twist input_velocity4_;
  geometry_msgs::msg::Twist input_velocity5_;
  geometry_msgs::msg::Twist input_velocity6_;
  // Output message
  geometry_msgs::msg::Twist output_velocity_;
};

class VelocitySwitcherNodeTest
    : public testing::TestWithParam<std::pair<int32_t, double> > {
 public:
  // Constructor
  VelocitySwitcherNodeTest() {}
  // Destructor
  ~VelocitySwitcherNodeTest() {}

 protected:
  virtual void SetUp();
  virtual void TearDown();

  // Function to issue test patterns according to the test_pattern_num number
  void PubTopic(const int32_t input_velocity_num);

  std::shared_ptr<TestNode> test_node_;
};
}  // namespace tmc_velocity_switcher
#endif  // TMC_VELOCITY_SWITCHER_VELOCITY_SWITCHER_TEST_HPP_
