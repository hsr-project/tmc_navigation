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
#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "../src/tmc_velocity_acceleration_limiter/velocity_acceleration_limiter.hpp"

namespace {
// Communication timeout duration [s]
constexpr double kConnectionTimeOut = 3.0;
// Output speed convergence timeout duration [s]
constexpr double kVelocityStableTimeOut = 5.0;
// Input speed [m/s, rad/s]
constexpr double kInputVelocity = 0.5;
// Maximum allowable ratio of expected acceleration/deceleration to output acceleration/deceleration value
constexpr double kAllowableRate = 1.05;
// Timeout judgment margin [s]
constexpr double kTimeoutMergin = 0.5;
// Time [s] to determine that output speed publishing has stopped
constexpr double kNotPublishThreshold = 0.5;
// Execution cycle [hz]
constexpr double kTestRate = 100.0;
// Service timeout duration [s]
constexpr double kTimeoutService = 5.0;

/// Parameter settings
constexpr double kLinearAccelerationLimit = 1.0;
constexpr double kLinearDecelerationLimit = 2.0;
constexpr double kAngularAccelerationLimit = 2.0;
constexpr double kAngularDecelerationLimit = 4.0;
constexpr double kInputVelocityTimeout = 0.2;
}  // anonymous namespace

namespace tmc_velocity_acceleration_limiter {
using std::placeholders::_1;

// Load parameters from yaml file
void LoadParameterFromYaml(std::shared_ptr<rclcpp::Node> node, const std::string& yaml_directory,
                           const std::string& yaml_name) {
  const std::string yaml_path = yaml_directory + yaml_name;
  // Load yaml and generate ParameterMap
  const rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_params_t* yaml_params = rcl_yaml_node_struct_init(allocator);
  rcl_parse_yaml_file(yaml_path.c_str(), yaml_params);
  const rclcpp::ParameterMap yaml_param_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);
  // Set ROS parameters to the node
  const std::string parameter_space = "/" + std::string(node->get_name());
  auto iter = yaml_param_map.find(parameter_space);
  for (auto& param : iter->second) {
    node->declare_parameter(param.get_name(), param.get_type());
    node->set_parameter(param);
  }
}

/// Check if the speed on the 2D plane is identical
bool Is2DVelocitySame(const geometry_msgs::msg::Twist& a, const geometry_msgs::msg::Twist& b) {
  return (std::abs(a.linear.x - b.linear.x) < std::numeric_limits<double>::epsilon() &&
          std::abs(a.linear.y - b.linear.y) < std::numeric_limits<double>::epsilon() &&
          std::abs(a.angular.z - b.angular.z) < std::numeric_limits<double>::epsilon());
}

/// Check if acceleration/deceleration is limited
/// @param [in] prev_vel Speed before acceleration/deceleration
/// @param [in] next_vel Speed after acceleration/deceleration
/// @param [in] dt Acceleration/deceleration time
/// @param [in] limit Acceleration/deceleration limit
/// @return Whether the limit is applied
bool IsAccelerationLimited(const double prev_vel, const double next_vel, const double dt, const double limit) {
  if (std::abs(next_vel - prev_vel) < std::numeric_limits<double>::epsilon()) return true;
  const double dv = (next_vel - prev_vel) / dt;
  return (std::abs(dv) / limit < kAllowableRate);
}

class TestNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options), rate_(kTestRate) {}

  /// Initialization
  void Init() {
    // Set up test subscriber/publisher
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/acceleration_limited_velocity", 10,
        std::bind(&TestNode::VelocityCallback, this, _1));
    pub_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>("command_velocity", 1);

    // Set up test service client
    enable_limit_ = this->create_client<std_srvs::srv::Empty>("/velocity_acceleration_limiter/enable");
    enable_limit_->wait_for_service(std::chrono::milliseconds(static_cast<int32_t>(kConnectionTimeOut * 1000)));
    disable_limit_ = this->create_client<std_srvs::srv::Empty>("/velocity_acceleration_limiter/disable");
    disable_limit_->wait_for_service(std::chrono::milliseconds(static_cast<int32_t>(kConnectionTimeOut * 1000)));

    // Set main process execution timer
    node_action_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int32_t>(1000 / kTestRate)),
        std::bind(&TestNode::NodeActionTimerCallback, this));
  }

  /// Wait until the target node and Pub/Sub are connected
  /// @return Whether the connection is complete
  bool WaitForConnectionEstablished() {
    return WaitUntil([&]() {
      return (sub_velocity_->get_publisher_count() != 0 &&
              pub_velocity_->get_subscription_count() != 0);
    }, kConnectionTimeOut);
  }

  /// Start publishing input speed
  void StartPublishVelocity(const geometry_msgs::msg::Twist& velocity) {
    publish_velocity_ = velocity;
    is_publish_velocity_ = true;
  }

  /// Stop publishing input speed
  void StopPublishVelocity() {
    is_publish_velocity_ = false;
  }

  /// Execute acceleration/deceleration limit enable service
  void CallStartService() {
    CallEmptyService(enable_limit_);
  }

  /// Execute acceleration/deceleration limit disable service
  void CallStopService() {
    CallEmptyService(disable_limit_);
  }

  /// Initialize output speed buffer
  void CrearOutputVelocities() {
    output_velocities_.clear();
  }

  /// Clear output speed buffer except for the last one
  void CrearOutputVelocitiesExceptLast() {
    output_velocities_.erase(output_velocities_.begin(), output_velocities_.end() - 1);
  }

  std::vector<geometry_msgs::msg::TwistStamped> output_velocities() const { return output_velocities_; }

  /// Wait until the expected speed is output
  /// @param [in] expected_velocity Expected speed
  /// @param [in] timeout Waiting timeout duration
  /// @return Whether the expected speed was achieved
  bool WaitForExpectedVelocity(const geometry_msgs::msg::Twist& expected_velocity, const double timeout) {
    return WaitUntil([&]() {
        return (!output_velocities_.empty() && Is2DVelocitySame(output_velocities_.back().twist, expected_velocity));
    }, timeout);
  }

  /// Wait until speed output stops
  /// @param [in] timeout Waiting timeout duration
  /// @return Whether speed output has stopped
  bool WaitForNotVelocityPublished(const double timeout) {
    return WaitUntil([&]() {
        return (!output_velocities_.empty() &&
                (rclcpp::Clock(RCL_ROS_TIME).now() - output_velocities_.back().header.stamp).seconds()
                 > kNotPublishThreshold);
    }, timeout);
  }

  /// Whether speed output is not limited
  /// @param [in] input_from Input speed before change
  /// @param [in] input_to Input speed after change
  /// @return Whether speed output is not limited
  bool IsTwistAccelerationNotLimited(const geometry_msgs::msg::Twist& input_from,
                                     const geometry_msgs::msg::Twist& input_to) {
    for (uint32_t i = 0; i < output_velocities_.size() - 1; ++i) {
      const geometry_msgs::msg::Twist prev = output_velocities_.at(i).twist;
      const geometry_msgs::msg::Twist next = output_velocities_.at(i + 1).twist;
      // Determine that there is no limit if consecutive output speeds jump in the same way as input speed changes
      if (Is2DVelocitySame(prev, input_from) && Is2DVelocitySame(next, input_to)) return true;
    }
    return false;
  }

  /// Whether speed output is limited by acceleration
  bool IsTwistAccelerationLimited() {
    // Determine if consecutive output speeds are monotonically increasing
    for (uint32_t i = 0; i < output_velocities_.size() - 1; ++i) {
      const geometry_msgs::msg::Twist prev = output_velocities_.at(i).twist;
      const geometry_msgs::msg::Twist next = output_velocities_.at(i + 1).twist;
      if ((std::abs(next.linear.x) - std::abs(prev.linear.x) < 0.0) ||
          (std::abs(next.linear.y) - std::abs(prev.linear.y) < 0.0) ||
          (std::abs(next.angular.z) - std::abs(prev.angular.z) < 0.0)) {
        return false;
      }
    }
    // Determine if the rate of change of output speed is within the acceleration limit
    const geometry_msgs::msg::TwistStamped start = output_velocities_.front();
    const geometry_msgs::msg::TwistStamped end = output_velocities_.back();
    const double dt = (rclcpp::Time(end.header.stamp) - rclcpp::Time(start.header.stamp)).seconds();
    if (dt < std::numeric_limits<double>::epsilon()) return false;
    return (IsAccelerationLimited(start.twist.linear.x, end.twist.linear.x, dt, kLinearAccelerationLimit) &&
            IsAccelerationLimited(start.twist.linear.y, end.twist.linear.y, dt, kLinearAccelerationLimit) &&
            IsAccelerationLimited(start.twist.angular.z, end.twist.angular.z, dt, kAngularAccelerationLimit));
  }

  /// Whether speed output is limited by deceleration
  bool IsTwistDecelerationLimited() {
    // Determine if consecutive output speeds are monotonically decreasing
    for (uint32_t i = 0; i < output_velocities_.size() - 1; ++i) {
      const geometry_msgs::msg::Twist prev = output_velocities_.at(i).twist;
      const geometry_msgs::msg::Twist next = output_velocities_.at(i + 1).twist;
      if ((std::abs(next.linear.x) - std::abs(prev.linear.x) > 0.0) ||
          (std::abs(next.linear.y) - std::abs(prev.linear.y) > 0.0) ||
          (std::abs(next.angular.z) - std::abs(prev.angular.z) > 0.0)) {
        return false;
      }
    }
    // Determine if the rate of change of output speed is within the deceleration limit
    const geometry_msgs::msg::TwistStamped start = output_velocities_.front();
    const geometry_msgs::msg::TwistStamped end = output_velocities_.back();
    const double dt = (rclcpp::Time(end.header.stamp) - rclcpp::Time(start.header.stamp)).seconds();
    if (dt < std::numeric_limits<double>::epsilon()) return false;
    return (IsAccelerationLimited(start.twist.linear.x, end.twist.linear.x, dt, kLinearDecelerationLimit) &&
            IsAccelerationLimited(start.twist.linear.y, end.twist.linear.y, dt, kLinearDecelerationLimit) &&
            IsAccelerationLimited(start.twist.angular.z, end.twist.angular.z, dt, kAngularDecelerationLimit));
  }

 private:
  /// Test target node output speed callback
  void VelocityCallback(const geometry_msgs::msg::Twist& msg) {
    geometry_msgs::msg::TwistStamped ouput_velocity;
    ouput_velocity.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    ouput_velocity.twist = msg;
    output_velocities_.push_back(ouput_velocity);
  }

  /// Periodic processing callback
  void NodeActionTimerCallback() {
    if (is_publish_velocity_) {
      pub_velocity_->publish(publish_velocity_);
    }
  }

  /// Wait until some condition is met
  bool WaitUntil(std::function<bool()> condition_function, const double timeout_sec) {
    // Error check for arguments
    if (!condition_function) {
      throw std::invalid_argument("Function for waiting is empty.");
    }
    if (timeout_sec < 0.0) {
      throw std::invalid_argument("Timeout must have fully value");
    }

    const rclcpp::Time end_time = rclcpp::Clock(RCL_ROS_TIME).now() + rclcpp::Duration::from_seconds(timeout_sec);
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      if (condition_function()) return true;
      if (rclcpp::Clock(RCL_ROS_TIME).now() >= end_time) break;
      rate_.sleep();
    }
    return false;
  }

  /// Execute service
  void CallEmptyService(rclcpp::Client<std_srvs::srv::Empty>::SharedPtr& client) {
    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result_future = client->async_send_request(empty_request);
    const rclcpp::Time start = rclcpp::Clock(RCL_ROS_TIME).now();
    while ((rclcpp::Clock(RCL_ROS_TIME).now() - start).seconds() < kTimeoutService) {
      rclcpp::spin_some(shared_from_this());
      auto status = result_future.wait_for(std::chrono::milliseconds(10));
      if (status == std::future_status::ready) {
        return;
      }
      rate_.sleep();
    }
    throw std::runtime_error("Can not call service.");
  }

  // Input speed publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity_;
  // Output speed subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_velocity_;
  // Acceleration/deceleration limit enable service client
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr enable_limit_;
  // Acceleration/deceleration limit disable service client
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr disable_limit_;
  // Test target node input speed
  geometry_msgs::msg::Twist input_velocity_;
  // Test target node output speed buffer
  std::vector<geometry_msgs::msg::TwistStamped> output_velocities_;
  // Main process timer
  rclcpp::TimerBase::SharedPtr node_action_timer_;
  // Whether to periodically send speed
  bool is_publish_velocity_;
  // Periodic transmission speed
  geometry_msgs::msg::Twist publish_velocity_;
  // Execution cycle
  rclcpp::Rate rate_;
};


/// Test fixture
class VelocityAccelerationLimiterTest : public ::testing::Test {
 public:
  VelocityAccelerationLimiterTest() {}

 protected:
  virtual void SetUp() {
    // Generate test node
    test_node_ = std::make_shared<TestNode>();
    test_node_->Init();
    // Wait until linked with publisher/subscriber
    if (!test_node_->WaitForConnectionEstablished()) {
      RCLCPP_FATAL(rclcpp::get_logger("velocity_acceleration_limiter_test"), "Can not link to test target.");
      exit(EXIT_FAILURE);
    }
    // Input speed
    input_velocity_.linear.x = kInputVelocity;
    input_velocity_.linear.y = kInputVelocity;
    input_velocity_.angular.z = kInputVelocity;
  }

  virtual void TearDown() {
    test_node_->StopPublishVelocity();
    test_node_->CrearOutputVelocities();
  }
  // Test node
  std::shared_ptr<TestNode> test_node_;
  // Test target node input speed
  geometry_msgs::msg::Twist input_velocity_;
};


/// Test that acceleration limits are applied according to parameters
/// Also test that acceleration limits can be turned ON and OFF via service
TEST_F(VelocityAccelerationLimiterTest, AccelerationLimit) {
  // Turn off functionality via service
  test_node_->CallStopService();
  test_node_->CrearOutputVelocities();
  // Start zero speed input and wait until output speed matches input speed
  test_node_->StartPublishVelocity(geometry_msgs::msg::Twist());
  EXPECT_TRUE(test_node_->WaitForExpectedVelocity(geometry_msgs::msg::Twist(), kVelocityStableTimeOut));
  test_node_->StopPublishVelocity();
  // To test speed transitions from here, clear all speeds up to this point except the last one
  test_node_->CrearOutputVelocitiesExceptLast();

  // Start speed input and wait until output speed matches input speed
  test_node_->StartPublishVelocity(input_velocity_);
  EXPECT_TRUE(test_node_->WaitForExpectedVelocity(input_velocity_, kVelocityStableTimeOut));
  test_node_->StopPublishVelocity();
  // Output speed is not limited by acceleration/deceleration
  EXPECT_TRUE(test_node_->IsTwistAccelerationNotLimited(geometry_msgs::msg::Twist(), input_velocity_));

  // Turn on functionality via service
  test_node_->CallStartService();
  test_node_->CrearOutputVelocities();
  // Start zero speed input and wait until output speed matches input speed
  test_node_->StartPublishVelocity(geometry_msgs::msg::Twist());
  EXPECT_TRUE(test_node_->WaitForExpectedVelocity(geometry_msgs::msg::Twist(), kVelocityStableTimeOut));
  test_node_->StopPublishVelocity();

  // To test speed transitions from here, clear all speeds up to this point except the last one
  test_node_->CrearOutputVelocitiesExceptLast();
  // Start speed input and wait until output speed matches input speed
  test_node_->StartPublishVelocity(input_velocity_);
  // Acceleration completes within the expected time calculated from parameters + margin
  const double expected_accel_time = kInputVelocity / std::min(kLinearAccelerationLimit, kAngularAccelerationLimit);
  EXPECT_TRUE(test_node_->WaitForExpectedVelocity(input_velocity_, expected_accel_time + kTimeoutMergin));
  test_node_->StopPublishVelocity();
  // Output speed is limited by acceleration
  EXPECT_TRUE(test_node_->IsTwistAccelerationLimited());
}

/// Test that deceleration limits are applied according to parameters
/// Also test that deceleration limits can be turned ON and OFF via service
TEST_F(VelocityAccelerationLimiterTest, DecelerationLimit) {
  // Turn off functionality via service
  test_node_->CallStopService();
  test_node_->CrearOutputVelocities();
  // Start speed input and wait until output speed matches input speed
  test_node_->StartPublishVelocity(input_velocity_);
  EXPECT_TRUE(test_node_->WaitForExpectedVelocity(input_velocity_, kVelocityStableTimeOut));
  test_node_->StopPublishVelocity();
  // To test speed transitions from here, clear all speeds up to this point except the last one
  test_node_->CrearOutputVelocitiesExceptLast();
  // Switch input to zero speed and wait until output speed matches input speed
  test_node_->StartPublishVelocity(geometry_msgs::msg::Twist());
  EXPECT_TRUE(test_node_->WaitForExpectedVelocity(geometry_msgs::msg::Twist(), kVelocityStableTimeOut));
  test_node_->StopPublishVelocity();
  // Output speed is not limited by acceleration/deceleration
  EXPECT_TRUE(test_node_->IsTwistAccelerationNotLimited(input_velocity_, geometry_msgs::msg::Twist()));

  // Turn on functionality via service
  test_node_->CallStartService();
  test_node_->CrearOutputVelocities();
  // Start speed input and wait until output speed matches input speed
  test_node_->StartPublishVelocity(input_velocity_);
  EXPECT_TRUE(test_node_->WaitForExpectedVelocity(input_velocity_, kVelocityStableTimeOut));
  test_node_->StopPublishVelocity();
  // To test speed transitions from here, clear all speeds up to this point except the last one
  test_node_->CrearOutputVelocitiesExceptLast();
  // Switch input to zero speed and wait until output speed matches input speed
  test_node_->StartPublishVelocity(geometry_msgs::msg::Twist());
  // Deceleration completes within the expected time calculated from parameters + margin
  const double expected_decel_time = kInputVelocity / std::min(kLinearDecelerationLimit, kAngularDecelerationLimit);
  EXPECT_TRUE(test_node_->WaitForExpectedVelocity(geometry_msgs::msg::Twist(), expected_decel_time + kTimeoutMergin));
  test_node_->StopPublishVelocity();
  // Output speed is limited by deceleration
  EXPECT_TRUE(test_node_->IsTwistDecelerationLimited());
}

/// Test that speed output stops if input speed is interrupted for a certain period
/// Decelerate and finally publish speed 0
TEST_F(VelocityAccelerationLimiterTest, PublishNoVelocityWhenInputVelcityTimeout) {
  // Start speed input and wait until output speed matches input speed
  test_node_->StartPublishVelocity(input_velocity_);
  EXPECT_TRUE(test_node_->WaitForExpectedVelocity(input_velocity_, kVelocityStableTimeOut));
  test_node_->CrearOutputVelocities();
  // Stop input speed
  test_node_->StopPublishVelocity();
  // Speed output stops within the time calculated from parameters + margin
  const double expected_timeout =
      kInputVelocityTimeout +
      kInputVelocity / std::min(kLinearDecelerationLimit, kAngularDecelerationLimit) +
      kNotPublishThreshold;
  EXPECT_TRUE(test_node_->WaitForNotVelocityPublished(expected_timeout + kTimeoutMergin));
  // Output speed is limited by deceleration
  EXPECT_TRUE(test_node_->IsTwistDecelerationLimited());
  // Final speed is 0
  const std::vector<geometry_msgs::msg::TwistStamped> output_velocities = test_node_->output_velocities();
  EXPECT_TRUE(Is2DVelocitySame(geometry_msgs::msg::Twist(), output_velocities.back().twist));
}
}  // namespace tmc_velocity_acceleration_limiter

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);

  // Generate marker_based_localizer node
  auto velocity_acceleration_limiter_node =
      std::make_shared<tmc_velocity_acceleration_limiter::VelocityAccelerationLimiter>(option);
  // Read parameters from yaml
  const std::string yaml_directory = ament_index_cpp::get_package_share_directory("tmc_velocity_acceleration_limiter")
  + "/test/parameter/";
  LoadParameterFromYaml(velocity_acceleration_limiter_node, yaml_directory, "velocity_acceleration_limiter-test.yaml");
  velocity_acceleration_limiter_node->Init();

  // Start a thread to spin
  auto velocity_acceleration_limiter_node_thread = std::make_shared<std::thread>([&]() {
    try {
      rclcpp::spin(velocity_acceleration_limiter_node);
    } catch (const std::exception& e) {
      // Exceptions may occur during shutdown
      std::cout << "velocity_acceleration_limiter_node : " << e.what() << std::endl;
    }
  });
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  velocity_acceleration_limiter_node_thread->join();
  velocity_acceleration_limiter_node.reset();

  return result;
}
