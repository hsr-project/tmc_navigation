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
#ifndef TMC_ODOMETRY_SWITCHER_SUBSCRIBE_RATE_CHECKER_HPP_
#define TMC_ODOMETRY_SWITCHER_SUBSCRIBE_RATE_CHECKER_HPP_
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.h>

namespace tmc_odometry_switcher {
using std::placeholders::_1;
using std::placeholders::_2;
/// Reception cycle check class
/// Drop-off occurs with spin_some, so the cycle cannot be accurately checked.
/// Start separately from the test node and check the reception cycle.
class SubscribeRateChecker : public rclcpp::Node {
 public:
  explicit SubscribeRateChecker(const rclcpp::NodeOptions& options)
      : Node("subscribe_rate_checker", options), is_start_(false), subscribe_count_(0) {}

  void Init() {
    // publisher
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("switched_odom_subscribe_rate", 10);
    // subscriber
    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "switched_odom", 30, std::bind(&SubscribeRateChecker::OdometryCallback, this, _1));
    // srv
    start_server_ =
        this->create_service<std_srvs::srv::Empty>("start_check_rate",
        std::bind(&SubscribeRateChecker::StartCheckRateService, this, _1, _2));
    stop_server_ =
        this->create_service<std_srvs::srv::Empty>("stop_check_rate",
        std::bind(&SubscribeRateChecker::StopCheckRateService, this, _1, _2));
  }
  ~SubscribeRateChecker() {}

 private:
  // Cycle check start service
  void StartCheckRateService(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res) {
    subscribe_count_ = 0;
    start_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
    is_start_ = true;
  }
  // Cycle check stop service
  void StopCheckRateService(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res) {
    is_start_ = false;
  }

  // Odometry callback
  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (is_start_) {
      subscribe_count_++;
      rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
      std_msgs::msg::Float64 rate;
      rate.data = static_cast<double>(subscribe_count_) / (now - start_time_).seconds();
      publisher_->publish(rate);
    }
  }

  // Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
  // Publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  // Start service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_server_;
  // Stop service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_server_;
  // Reception start time
  rclcpp::Time start_time_;
  // Reception count
  uint32_t subscribe_count_;
  // Start
  bool is_start_;
};
}  // namespace tmc_odometry_switcher
#endif /*TMC_ODOMETRY_SWITCHER_SUBSCRIBE_RATE_CHECKER_HPP_*/
