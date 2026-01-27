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
/// @file safety_velocity_limiter.cpp
/// @brief Virtual bumper function
#include "safety_velocity_limiter.hpp"
#include <math.h>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "param.hpp"

namespace {
// node name
const char* kNodeName = "safety_velocity_limiter";
// service name
const char* const kStartServiceName = "~/start";                        // Function On service name
const char* const kStopServiceName = "~/stop";                          // Function Off service name
const char* const kSwitchBumperSetServiceName = "~/switch_bumper_set";  // Bumper set switch service name
const char* const kResetToDefaultServiceName = "~/reset_to_default";    // Service name to reset to default settings
const char* const kGetCurrentSettingServiceName = "~/get_current_setting";  // Service name to get current settings
// topic name
const char* kTopicInputVelocity = "input_velocity";       // Input velocity topic name
const char* kTopicOutputVelocity = "output_velocity";     // Output velocity topic name
const char* kTopicSlowingDown = "slowing_down";           // Sudden deceleration detection topic name
const char* kTopicObservedObstaclePose = "~/observed_obstacle_pose";  // Speed limit factor coordinates topic name
const char* kTopicRatio = "~/ratio";  // Deceleration ratio topic name
const char* kTopicZeroVelocity = "zero_velocity";        // Zero velocity detection topic name
// ROS parameter name
const char* kEnableFunction = "enable_function";                                 // Function enable/disable
const char* kDefaultBumperSet = "default_bumper_set";                            // Default bumper set
const char* kVirtualBumpers = "virtual_bumpers";                                 // Virtual bumper definition
const char* kSlowdownVelocityThreshold = "slowdown_velocity_threshold";          // Minimum speed for sudden deceleration detection [m/s]
const char* kSlowdownDecelerationThreshold = "slowdown_deceleration_threshold";  // Acceleration considered as sudden deceleration [m/s^2]
const char* kSlowdownDetectionTime = "slowdown_detection_time";                  // Sudden deceleration judgment time [s]
const char* kMaximumAcceleration = "maximum_acceleration";                       // Maximum acceleration (during acceleration) [m/s^2]
const char* kMaximumDeceleration = "maximum_deceleration";                       // Maximum acceleration (during deceleration) [m/s^2]
const char* kTimeoutInterval = "timeout_interval";                               // Speed timeout judgment time [s]
// ROS parameter default value
const bool kEnableFunctionDef = false;                  // Function enable/disable
const double kSlowdownVelocityThresholdDef = 0.5;       // Minimum speed for sudden deceleration [m/s]
const double kSlowdownDecelerationThresholdDef = 2.0;   // Acceleration considered as sudden deceleration [m/s^2]
const double kSlowdownDetectionTimeDef = 0.0;           // Sudden deceleration judgment time [s]
const double kMaximumAccelerationDef = 0.5;             // Maximum acceleration (during acceleration) [m/s^2]
const double kMaximumDecelerationDef = 1.0;             // Maximum acceleration (during deceleration) [m/s^2]
const double kTimeoutIntervalDef = 0.5;                 // Speed timeout judgment time [s]
// Fixed parameter
const double kMinimumInterval = 0.001;  // Minimum value of speed update cycle [s]
const double kEpsilon = std::numeric_limits<double>::epsilon();

}  // anonymous namespace

namespace tmc_safety_velocity_limiter {
using std::placeholders::_1;
using std::placeholders::_2;
/// Constructor
VelocityLimiter::VelocityLimiter(const rclcpp::NodeOptions& options) :
    Node(kNodeName, options) {}

void VelocityLimiter::Init() {
  is_slowing_down_ = false;
  // Retrieve ros parameter
  UpdateParameters();
  // Register subscriber
  sub_velocity_ = this->create_subscription<Twist>(kTopicInputVelocity, 1,
      std::bind(&VelocityLimiter::VelocityCallback, this, _1));
  // Register publisher
  pub_velocity_ = this->create_publisher<Twist>(kTopicOutputVelocity, 1);
  pub_slowing_down_ = this->create_publisher<std_msgs::msg::Bool>(kTopicSlowingDown, 1);
  pub_observed_obstacle_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(kTopicObservedObstaclePose, 1);
  pub_ratio_ = this->create_publisher<std_msgs::msg::Float64>(kTopicRatio, 1);
  pub_zero_velocity_ = this->create_publisher<std_msgs::msg::Bool>(kTopicZeroVelocity, 1);
  // Function On, Off service
  start_service_ = this->create_service<std_srvs::srv::Empty>(kStartServiceName,
      std::bind(&VelocityLimiter::StartServiceCallback, this, _1, _2));
  stop_service_ = this->create_service<std_srvs::srv::Empty>(kStopServiceName,
      std::bind(&VelocityLimiter::StopServiceCallback, this, _1, _2));
  // Bumper set switch service
  switch_bumper_set_service_ = this->create_service<tmc_navigation_msgs::srv::SwitchBumperSet>(
      kSwitchBumperSetServiceName, std::bind(&VelocityLimiter::SwitchBumperSetServiceCallback, this, _1, _2));
  // Service to reset to default settings
  reset_to_default_service_ = this->create_service<std_srvs::srv::Empty>(
      kResetToDefaultServiceName, std::bind(&VelocityLimiter::ResetToDefaultServiceCallback, this, _1, _2));
  // Service to get current settings
  get_current_setting_service_ = this->create_service<tmc_navigation_msgs::srv::GetCurrentSetting>(
      kGetCurrentSettingServiceName, std::bind(&VelocityLimiter::GetCurrentSettingServiceCallback, this, _1, _2));

  // Obstacle initialization
  Obstacle::GetInstance()->Init(shared_from_this());
  // Occupancy initialization
  Occupancy::GetInstance()->Init(shared_from_this());
  // Sudden deceleration detection state initialization
  is_slowing_down_topic_.data = false;
  previous_operation_time_ = this->get_clock()->now();
  slowdown_status_update_time_ = this->get_clock()->now();
}

/// Destructor
VelocityLimiter::~VelocityLimiter() {}

/// Retrieve ROS PRAM
void VelocityLimiter::UpdateParameters() {
  // Default function On/Off state
  GetOptionalParam(shared_from_this(), kEnableFunction, default_enable_function_, kEnableFunctionDef);
  enable_function_ = default_enable_function_;
  // Sudden deceleration detection related
  GetOptionalParam(shared_from_this(), kSlowdownVelocityThreshold, slowdown_velocity_threshold_,
      kSlowdownVelocityThresholdDef);
  GetOptionalParam(shared_from_this(), kSlowdownDecelerationThreshold, slowdown_deceleration_threshold_,
      kSlowdownDecelerationThresholdDef);
  if (slowdown_deceleration_threshold_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Parameter [%s] is invalid (%lf). Use default value (%lf)",
        kSlowdownDecelerationThreshold, slowdown_deceleration_threshold_, kSlowdownDecelerationThresholdDef);
    slowdown_deceleration_threshold_ = kSlowdownDecelerationThresholdDef;
  }
  GetOptionalParam(shared_from_this(), kSlowdownDetectionTime, slowdown_detection_time_, kSlowdownDetectionTimeDef);

  // Retrieve maximum acceleration
  GetOptionalParam(shared_from_this(), kMaximumAcceleration, maximum_acceleration_, kMaximumAccelerationDef);
  if (maximum_acceleration_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Parameter [%s] is invalid (%lf). Use default value (%lf)",
        kMaximumAcceleration, maximum_acceleration_, kMaximumAccelerationDef);
    maximum_acceleration_ = kMaximumAccelerationDef;
  }
  GetOptionalParam(shared_from_this(), kMaximumDeceleration, maximum_deceleration_, kMaximumDecelerationDef);
  if (maximum_deceleration_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Parameter [%s] is invalid (%lf). Use default value (%lf)",
        kMaximumDeceleration, maximum_deceleration_, kMaximumDecelerationDef);
    maximum_deceleration_ = kMaximumDecelerationDef;
  }

  // Retrieve timeout judgment time
  GetOptionalParam(shared_from_this(), kTimeoutInterval, timeout_interval_, kTimeoutIntervalDef);
  if (timeout_interval_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Parameter [%s] is invalid (%lf). Use default value (%lf)",
        kTimeoutInterval, timeout_interval_, kTimeoutIntervalDef);
    timeout_interval_ = kTimeoutIntervalDef;
  }

  // Pass each parameter under "virtual_bumpers:" to the BumperSet constructor
  std::map<std::string, rclcpp::Parameter> virtual_bumpers;
  if (!GetGroupParam(shared_from_this(), kVirtualBumpers, virtual_bumpers)) {
    RCLCPP_WARN(this->get_logger(), "No virtual bumper definition.");
  } else {
    for (auto it = virtual_bumpers.begin(); it != virtual_bumpers.end(); ++it) {
      const size_t sbstr_index = it->first.find(".");
      if (sbstr_index == std::string::npos) {
        RCLCPP_INFO(this->get_logger(), "%s is invalid", it->first.c_str());
        continue;
      }
      const std::string bumper_set_name = it->first.substr(0, sbstr_index);
      if (bumper_sets_.find(bumper_set_name) == bumper_sets_.end()) {
        std::map<std::string, rclcpp::Parameter> bumper_set_param;
        GetGroupParam(virtual_bumpers, bumper_set_name, bumper_set_param);
        bumper_sets_[bumper_set_name] = BumperSet::Ptr(new BumperSet(bumper_set_param));
      }
    }
  }

  // Retrieve and set default bumper set
  if (!GetParam(shared_from_this(), kDefaultBumperSet, default_bumper_set_) ||
      !SwitchBumperSet(default_bumper_set_, std::vector<std::string>(0))) {
    throw std::runtime_error("Default bumper " + default_bumper_set_ + " is not registered in virtual_bumpers.");
  }
}

/// Velocity message callback
void VelocityLimiter::VelocityCallback(const TwistPtr msg) {
  rclcpp::Time now = this->get_clock()->now();
  Twist output_velocity = *msg;
  if (enable_function_) {
    // Speed update cycle to determine acceleration
    // Since the processing cycle depends on the upper node, determine from the actual callback invocation time
    rclcpp::Duration interval = now - previous_operation_time_;
    if (interval.seconds() < kMinimumInterval) {
      // Set a guard as interval may become zero
      interval = rclcpp::Duration::from_seconds(kMinimumInterval);
    } else if (interval.seconds() > timeout_interval_) {
      // If speed has timed out (stopped), calculate with previous value 0 and minimum cycle
      previous_velocity_ = Twist();
      interval = rclcpp::Duration::from_seconds(kMinimumInterval);
    }

    geometry_msgs::msg::PoseStamped observed_obstacle_pose;
    double velocity_limit_ratio = 1.0;
    // Speed limit
    if (current_bumper_set_.second->LimitVelocity(*msg, current_disable_bumpers_, output_velocity,
                                                  observed_obstacle_pose, velocity_limit_ratio)) {
      // If speed limit is applied, publish the coordinates of the obstacle that caused the limit
      pub_observed_obstacle_pose_->publish(observed_obstacle_pose);
    }
    double acceleration_limit_ratio = 1.0;
    // Acceleration limit
    LimitAcceleration(output_velocity, interval, acceleration_limit_ratio);
    // Sudden deceleration detection
    SlowdownDetection(output_velocity, interval);
    // Publish speed ratio
    std_msgs::msg::Float64 ratio_msg;
    ratio_msg.data = velocity_limit_ratio * acceleration_limit_ratio;
    pub_ratio_->publish(ratio_msg);
    // Zero speed detection by VirtualBumper
    std_msgs::msg::Bool is_zero_velocity_topic;
    is_zero_velocity_topic.data = DetectZeroVelocity(*msg, output_velocity);
    pub_zero_velocity_->publish(is_zero_velocity_topic);
  }
  // Publish speed
  pub_velocity_->publish(output_velocity);
  // Update previous speed and time
  previous_velocity_ = output_velocity;
  previous_operation_time_ = now;
}

/// Function On service
void VelocityLimiter::StartServiceCallback(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res) {
  enable_function_ = true;
}

/// Function Off service
void VelocityLimiter::StopServiceCallback(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res) {
  enable_function_ = false;
  // Reset sudden deceleration detection state while function is OFF
  ClearSlowdownDetectStatus();
}

/// Bumper set switch service
void VelocityLimiter::SwitchBumperSetServiceCallback(
    tmc_navigation_msgs::srv::SwitchBumperSet::Request::SharedPtr req,
    tmc_navigation_msgs::srv::SwitchBumperSet::Response::SharedPtr res) {
  std::vector<std::string> disable_bumpers;
  for (std::vector<std_msgs::msg::String>::iterator itr = req->disable_bumpers.begin();
       itr != req->disable_bumpers.end(); ++itr) {
    disable_bumpers.push_back(itr->data);
  }
  res->is_success = SwitchBumperSet(req->bumper_set.data, disable_bumpers);
}

/// Service to reset to default settings
void VelocityLimiter::ResetToDefaultServiceCallback(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res) {
  enable_function_ = default_enable_function_;
  if (!enable_function_) {
    // Reset sudden deceleration detection state while function is OFF
    ClearSlowdownDetectStatus();
  }
  // Since it is confirmed at startup that changes to default settings can be made, assume success and do not return results
  SwitchBumperSet(default_bumper_set_, std::vector<std::string>(0));
}

/// Service to get current settings
void VelocityLimiter::GetCurrentSettingServiceCallback(
    tmc_navigation_msgs::srv::GetCurrentSetting::Request::SharedPtr req,
    tmc_navigation_msgs::srv::GetCurrentSetting::Response::SharedPtr res) {
  res->enable_function = enable_function_;
  res->bumper_set.data = current_bumper_set_.first;
  for (std::vector<std::string>::iterator itr = current_disable_bumpers_.begin();
       itr != current_disable_bumpers_.end(); ++itr) {
    std_msgs::msg::String disable_bumper;
    disable_bumper.data = *itr;
    res->disable_bumpers.push_back(disable_bumper);
  }
}

/// Clear sudden deceleration detection state
void VelocityLimiter::ClearSlowdownDetectStatus() {
  is_slowing_down_ = false;
  previous_velocity_ = Twist();
  if (is_slowing_down_topic_.data) {
    is_slowing_down_topic_.data = false;
    pub_slowing_down_->publish(is_slowing_down_topic_);
  }
}

/// Bumper set switch
bool VelocityLimiter::SwitchBumperSet(
    const std::string& bumper_set, const std::vector<std::string>& disable_bumpers) {
  if (bumper_sets_.find(bumper_set) != bumper_sets_.end()) {
    RCLCPP_INFO(this->get_logger(), "SwitchBumperSet: Switch bumper set to \"%s\".", bumper_set.c_str());
    current_bumper_set_.first = bumper_set;
    current_bumper_set_.second = bumper_sets_[bumper_set];
    current_disable_bumpers_.clear();
    current_disable_bumpers_ = disable_bumpers;
  } else {
    RCLCPP_ERROR(this->get_logger(), "SwitchBumperSet: \"%s\" is not registered in virtual_bumpers.",
        bumper_set.c_str());
    return false;
  }
  return true;
}

/// Limit acceleration
void VelocityLimiter::LimitAcceleration(Twist& output_velocity, const rclcpp::Duration& interval, double& ratio) {
  double output_velocity_norm = sqrt(output_velocity.linear.x * output_velocity.linear.x +
                                     output_velocity.linear.y * output_velocity.linear.y);

  /// Do not limit if output speed is 0
  /// Do not limit if the reception interval from the previous speed is 0
  if (output_velocity_norm > std::numeric_limits<double>::epsilon() &&
      fabs(interval.seconds()) > std::numeric_limits<double>::epsilon()) {
    // Determine acceleration from the difference between previous speed and previous output time
    double previous_velocity_norm = sqrt(previous_velocity_.linear.x * previous_velocity_.linear.x +
                                         previous_velocity_.linear.y * previous_velocity_.linear.y);
    double acceleration = (output_velocity_norm - previous_velocity_norm) / interval.seconds();

    double limited_acceleration = acceleration;
    if (output_velocity_norm < previous_velocity_norm) {
      // In case of deceleration
      if (acceleration < -maximum_deceleration_) {
        limited_acceleration = -maximum_deceleration_;
      }
    } else {
      // In case of acceleration
      if (acceleration > maximum_acceleration_) {
        limited_acceleration = maximum_acceleration_;
      }
    }
    // If the current acceleration exceeds the set maximum acceleration, round the output speed
    // Limit X, Y, and rotation at the same ratio
    if (limited_acceleration != acceleration) {
      double limited_velocity_norm = previous_velocity_norm + limited_acceleration * interval.seconds();
      ratio = limited_velocity_norm / output_velocity_norm;
      output_velocity.linear.x *= ratio;
      output_velocity.linear.y *= ratio;
      output_velocity.angular.z *= ratio;
    }
  }
}

/// Sudden deceleration detection
void VelocityLimiter::SlowdownDetection(const Twist& output_velocity, const rclcpp::Duration& interval) {
  /// Do not judge sudden deceleration if the reception interval from the previous speed is 0
  if (fabs(interval.seconds()) < std::numeric_limits<double>::epsilon()) {
    return;
  }

  rclcpp::Time now = this->get_clock()->now();

  // Determine acceleration from the difference between previous speed and previous output time
  // Since only deceleration is targeted, determine acceleration from the absolute value of speed, not considering direction
  const double previous_velocity_norm = sqrt(previous_velocity_.linear.x * previous_velocity_.linear.x +
      previous_velocity_.linear.y * previous_velocity_.linear.y);
  const double output_velocity_norm = sqrt(output_velocity.linear.x * output_velocity.linear.x +
      output_velocity.linear.y * output_velocity.linear.y);
  const double deceleration = -(output_velocity_norm - previous_velocity_norm) / interval.seconds();

  // Sudden deceleration detection judgment
  // Previous speed is greater than the reference value, and deceleration is greater than the reference value
  const bool detected =
      ((previous_velocity_norm > slowdown_velocity_threshold_) && (deceleration > slowdown_deceleration_threshold_));

  // Record time if the judgment result differs from the previous one
  if ((detected && !is_slowing_down_) || (!detected && is_slowing_down_)) {
    is_slowing_down_ = detected;
    slowdown_status_update_time_ = now;
  }

  // Reflect in the topic if the detection state/non-detection state persists for a specified time
  const double duration = (now - slowdown_status_update_time_).seconds();
  if (detected) {
    if (!is_slowing_down_topic_.data && slowdown_detection_time_ <= duration) {
      is_slowing_down_topic_.data = true;
      pub_slowing_down_->publish(is_slowing_down_topic_);
    }
  } else {
    if (is_slowing_down_topic_.data && slowdown_detection_time_ <= duration) {
      is_slowing_down_topic_.data = false;
      pub_slowing_down_->publish(is_slowing_down_topic_);
    }
  }
}

/// Zero speed detection
bool VelocityLimiter::DetectZeroVelocity(const Twist& input_velocity, const Twist& output_velocity) {
  if ((fabs(input_velocity.linear.x) > kEpsilon || fabs(input_velocity.linear.y) > kEpsilon ||
       fabs(input_velocity.angular.z) > kEpsilon) &&
      (fabs(output_velocity.linear.x) <= kEpsilon && fabs(output_velocity.linear.y) <= kEpsilon &&
       fabs(output_velocity.angular.z) <= kEpsilon)) {
    return true;
  }
  return false;
}
}  // namespace tmc_safety_velocity_limiter
