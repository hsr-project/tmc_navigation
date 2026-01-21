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
/// @file safety_velocity_limiter.hpp
/// @brief Virtual bumper function
#ifndef TMC_SAFETY_VELOCITY_LIMITER_SAFETY_VELOCITY_LIMITER_HPP_
#define TMC_SAFETY_VELOCITY_LIMITER_SAFETY_VELOCITY_LIMITER_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tmc_navigation_msgs/srv/get_current_setting.hpp>
#include <tmc_navigation_msgs/srv/switch_bumper_set.hpp>
#include "bumper_set.hpp"
#include "common.hpp"
#include "obstacle.hpp"
#include "occupancy.hpp"

namespace tmc_safety_velocity_limiter {

/// Uses obstacle information, etc., to impose speed limits on the robot according to the situation
class VelocityLimiter : public rclcpp::Node {
 public:
  // Constructor
  explicit VelocityLimiter(const rclcpp::NodeOptions& options);
  // Initialization
  void Init();
  // Destructor
  ~VelocityLimiter();


 private:
  // Subscriber callback
  void VelocityCallback(const TwistPtr msg);
  // Parameter update
  void UpdateParameters();
  // Function On service
  void StartServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);
  // Function Off service
  void StopServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);
  // Bumper set switch service
  void SwitchBumperSetServiceCallback(tmc_navigation_msgs::srv::SwitchBumperSet::Request::SharedPtr req,
      tmc_navigation_msgs::srv::SwitchBumperSet::Response::SharedPtr res);
  // Service to reset to default settings
  void ResetToDefaultServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);
  // Service to get current settings
  void GetCurrentSettingServiceCallback(tmc_navigation_msgs::srv::GetCurrentSetting::Request::SharedPtr req,
      tmc_navigation_msgs::srv::GetCurrentSetting::Response::SharedPtr res);
  // Acceleration limit
  void LimitAcceleration(Twist& output_velocity, const rclcpp::Duration& interval, double& ratio);
  // Sudden deceleration detection
  void SlowdownDetection(const Twist& output_velocity, const rclcpp::Duration& interval);
  // Clear sudden deceleration detection state
  void ClearSlowdownDetectStatus();
  // Bumper set switch
  bool SwitchBumperSet(const std::string& bumper_set, const std::vector<std::string>& disable_bumpers);
  // Zero speed detection
  bool DetectZeroVelocity(const Twist& input_velocity, const Twist& output_velocity);

  // Subscriber / Publisher
  rclcpp::Subscription<Twist>::SharedPtr sub_velocity_;
  rclcpp::Publisher<Twist>::SharedPtr pub_velocity_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_slowing_down_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_observed_obstacle_pose_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ratio_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_zero_velocity_;

  // Parameters
  double slowdown_velocity_threshold_;      // Minimum speed for sudden deceleration detection [m/s]
  double slowdown_deceleration_threshold_;  // Acceleration considered as sudden deceleration [m/s^2]
  double slowdown_detection_time_;          // Sudden deceleration detection time [s]
  double maximum_acceleration_;             // Maximum acceleration (during acceleration) [m/s^2]
  double maximum_deceleration_;             // Maximum acceleration (during deceleration) [m/s^2]
  double timeout_interval_;                 // Speed timeout detection time [s]
  bool default_enable_function_;            // Default value for function enable/disable
  std::string default_bumper_set_;          // Default value for bumper set name

  // Function On/Off flag
  bool enable_function_;
  // Function On service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service_;
  // Function Off service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
  // Bumper set switch service
  rclcpp::Service<tmc_navigation_msgs::srv::SwitchBumperSet>::SharedPtr switch_bumper_set_service_;
  // Service to reset to default settings
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_to_default_service_;
  // Service to get current settings
  rclcpp::Service<tmc_navigation_msgs::srv::GetCurrentSetting>::SharedPtr get_current_setting_service_;
  // Registered bumper sets
  std::map<std::string, BumperSet::Ptr> bumper_sets_;

  // Currently selected bumper set
  std::pair<std::string, BumperSet::Ptr> current_bumper_set_;
  // Currently set invalid bumper list
  std::vector<std::string> current_disable_bumpers_;

  // Output speed at the last control
  geometry_msgs::msg::Twist previous_velocity_;
  // Time at the last control
  rclcpp::Time previous_operation_time_;
  // Whether sudden deceleration is being detected
  bool is_slowing_down_;
  // Time when sudden deceleration detection/non-detection switched
  rclcpp::Time slowdown_status_update_time_;
  // State of the output topic
  std_msgs::msg::Bool is_slowing_down_topic_;
};

}  // namespace tmc_safety_velocity_limiter

#endif  // TMC_SAFETY_VELOCITY_LIMITER_SAFETY_VELOCITY_LIMITER_HPP_
