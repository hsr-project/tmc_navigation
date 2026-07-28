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
#include <math.h>

#include <chrono>
#include <limits>
#include <angles/angles.h>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_srvs/srv/empty.hpp>
// TODO(syuuhei_shiro) tmc_rostest_utilsのROS2化
// #include <tmc_rostest_utils/rostest_utils.hpp>

#include "test_utils.hpp"
#include "../src/base_velocity_adjuster_node.hpp"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
namespace {
// Communication timeout duration [sec]
constexpr double kTimeOut = 3.0;
// Output velocity stabilization timeout duration [sec]
constexpr double kVelocityStableTimeOut = 1.0;
// Obstacle topic publishing margin [sec]
constexpr double kMerginTimeToPublishObstacle = 2.0;
// Update cycle [hz]
constexpr double kTestRate = 100.0;
// Input velocity
constexpr double kInputVelocityX = 1.0;
constexpr double kInputVelocityY = 0.0;
constexpr double kInputVelocityT = 0.0;
// Obstacle placement angle step [rad]
constexpr double kObstacleDirectionStep = (7.5 / 180.0 * M_PI);
// Obstacle radius [m]
constexpr double kObstacleRadius = 0.4;
// Number of obstacle points
constexpr uint32_t kObstaclePointNum = 10;
// Obstacle direction relative to the front of the path [rad]
constexpr double kObstacleDirection = (15.0 / 180.0 * M_PI);
// Obstacle cloud frame. By setting it the same as the cart frame, tf publishing can be omitted.
constexpr const char* const kObstacleFrame = "base_link";
// Time interval for interference prediction [s]
constexpr double kEstimationTime = 1.0;
}  // anonymous namespace

namespace tmc_base_velocity_adjuster {
using std::placeholders::_1;

/// Test node class
class TestNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options) {}

  /// Initialization
  void Init() {
    // Test subscriber and publisher settings
    pub_obstacle_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle", 1);
    pub_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>("command_velocity", 1);
    sub_output_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "adjusted_velocity", 1, std::bind(&TestNode::VelocityCallback, this, _1));

    // Test service client settings
    enable_velocity_adjust_ = this->create_client<std_srvs::srv::Empty>("base_velocity_adjuster/enable");
    enable_velocity_adjust_->wait_for_service(std::chrono::milliseconds(static_cast<int32_t>(kTimeOut * 1000)));
    disable_velocity_adjust_ = this->create_client<std_srvs::srv::Empty>("base_velocity_adjuster/disable");
    disable_velocity_adjust_->wait_for_service(std::chrono::milliseconds(static_cast<int32_t>(kTimeOut * 1000)));

    // Main process execution timer setup
    node_action_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int32_t>(1000 / kTestRate)),
        std::bind(&TestNode::NodeActionTimerCallback, this));
  }

  /// Publish obstacle cloud
  void PublishObstacle(const PointCloud& obstacle) {
    sensor_msgs::msg::PointCloud2 published_obstacle;
    pcl::toROSMsg(obstacle, published_obstacle);
    published_obstacle.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    published_obstacle.header.frame_id = kObstacleFrame;
    pub_obstacle_->publish(published_obstacle);
  }

  void StartPublishVelocity(const geometry_msgs::msg::Twist& velocity) {
    publish_velocity_ = velocity;
    is_publish_velocity_ = true;
  }

  void StopPublishVelocity() {
    is_publish_velocity_ = false;
  }

  /// Wait until the connection with the target node is established
  bool WaitForConnectionEstablished() {
    return WaitUntil(nullptr, [&]() {
      return (sub_output_velocity_->get_publisher_count() > 0 &&
              pub_velocity_->get_subscription_count() > 0 &&
              pub_obstacle_->get_subscription_count() > 0);
    }, kTimeOut);
  }

  /// Wait for the output velocity to reach the expected value
  bool WaitForExpectedVelocity(const geometry_msgs::msg::Twist& expected_velocity) {
    return WaitUntil(nullptr, [&]() {
      return (velocity_subscribed_ && Is2DVelocitySame(output_velocity_, expected_velocity));
    }, kVelocityStableTimeOut);
  }

  /// Turn ON velocity correction function
  void EnableVelocityAdjust() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    enable_velocity_adjust_->async_send_request(request);
  }

  /// Turn OFF velocity correction function
  void DisableVelocityAdjust() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    disable_velocity_adjust_->async_send_request(request);
  }
  /// Accessor
  geometry_msgs::msg::Twist output_velocity() const { return output_velocity_; }
  bool velocity_subscribed() const { return velocity_subscribed_; }

 private:
  void NodeActionTimerCallback() {
    if (is_publish_velocity_) {
      pub_velocity_->publish(publish_velocity_);
    }
  }

  /// Callback for test target node output velocity
  void VelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    output_velocity_ = *msg;
    velocity_subscribed_ = true;
  }

  // Input velocity output
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity_;
  // Obstacle information publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_;
  // Test target node output velocity subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_output_velocity_;
  // Velocity correction function ON service client
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr enable_velocity_adjust_;
  // Velocity correction function OFF service client
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr disable_velocity_adjust_;
  // Periodic transmission velocity
  geometry_msgs::msg::Twist publish_velocity_;
  // Test target node output velocity
  geometry_msgs::msg::Twist output_velocity_;
  // Subscription flag
  bool velocity_subscribed_;
  // Main process timer
  rclcpp::TimerBase::SharedPtr node_action_timer_;
  // Whether to periodically transmit velocity
  bool is_publish_velocity_;
};

/// BaseVelocityAdjuster node test fixture
class BaseVelocityAdjusterNodeTest : public ::testing::Test {
 public:
    BaseVelocityAdjusterNodeTest() {}

 protected:
  virtual void SetUp() {
    // Test node creation
    test_node_ = std::make_shared<TestNode>();
    test_node_->Init();
    // Create a thread for spinning and detach it
    auto test_node_thread = std::make_shared<std::thread>([&]() {
        rclcpp::spin(test_node_);
        });
    test_node_thread->detach();
    // Input velocity
    input_velocity_.linear.x = kInputVelocityX;
    input_velocity_.linear.y = kInputVelocityY;
    input_velocity_.angular.z = kInputVelocityT;

    // Whether connected to the target node
    ASSERT_TRUE(test_node_->WaitForConnectionEstablished());
    // The BaseVelocityOptimizer class waits for the previous state, so publish an empty obstacle to reset
    const PointCloud empty_obstacle;
    test_node_->PublishObstacle(empty_obstacle);
  }

  virtual void TearDown() {
    test_node_->StopPublishVelocity();
  }

  std::shared_ptr<TestNode> test_node_;
  // Test target node input velocity
  geometry_msgs::msg::Twist input_velocity_;
};
/// Service test
/// Velocity correction function can be turned ON and OFF via service
TEST_F(BaseVelocityAdjusterNodeTest, SwitchVelocityAdjustService) {
  // setup
  // Set obstacle on the left side of the path
  PointCloud obstacle;
  const double obstacle_x =
      input_velocity_.linear.x * kEstimationTime * cos(kObstacleDirection) -
      input_velocity_.linear.y * kEstimationTime * sin(kObstacleDirection);
  const double obstacle_y =
      input_velocity_.linear.x * kEstimationTime * sin(kObstacleDirection) +
      input_velocity_.linear.y * kEstimationTime * cos(kObstacleDirection);
  AddCircularObstacle(obstacle_x, obstacle_y, kObstacleRadius, kObstaclePointNum, obstacle);

  // exercise
  // By default, the velocity correction function is ON, so start testing by turning it OFF via service
  // Turn OFF velocity correction function via service
  test_node_->DisableVelocityAdjust();
  // Publish obstacle
  test_node_->PublishObstacle(obstacle);
  // Start publishing input velocity
  test_node_->StartPublishVelocity(input_velocity_);

  // Wait for the output to reach the expected value
  EXPECT_TRUE(test_node_->WaitForExpectedVelocity(input_velocity_));

  // verify
  // Ensure input and output match (already verified with WaitForExpectedVelocity, but re-evaluated to record actual values)
  EXPECT_DOUBLE_EQ(input_velocity_.linear.x, test_node_->output_velocity().linear.x);
  EXPECT_DOUBLE_EQ(input_velocity_.linear.y, test_node_->output_velocity().linear.y);
  EXPECT_DOUBLE_EQ(input_velocity_.angular.z, test_node_->output_velocity().angular.z);

  // exercise
  // Test turning ON the velocity correction function via service after it was turned OFF
  // Turn ON velocity correction function via service
  test_node_->EnableVelocityAdjust();
  // Publish obstacle
  test_node_->PublishObstacle(obstacle);

  // Wait for the output to reach the expected value
  const double input_direction = atan2(input_velocity_.linear.y, input_velocity_.linear.x);
  double direction_diff = 0.0;
  EXPECT_TRUE(WaitUntil(nullptr, [&]() {
      const double output_direction = atan2(test_node_->output_velocity().linear.y,
          test_node_->output_velocity().linear.x);
      direction_diff = angles::shortest_angular_distance(input_direction, output_direction);
      return (test_node_->velocity_subscribed() && direction_diff < 0.0);
    }, kVelocityStableTimeOut));

  // verify
  // Ensure the path is corrected to the right to avoid the obstacle on the left
  EXPECT_LT(direction_diff, 0.0);
  // Ensure turning velocity is not corrected
  EXPECT_DOUBLE_EQ(input_velocity_.angular.z, test_node_->output_velocity().angular.z);
}

/// Velocity correction test
/// No correction if there are no obstacles
TEST_F(BaseVelocityAdjusterNodeTest, NotAdjustedForNoObstacle) {
  // setup
  // Turn ON velocity correction function via service
  test_node_->EnableVelocityAdjust();

  // exercise
  // Publish empty obstacle
  PointCloud obstacle;
  test_node_->PublishObstacle(obstacle);
  // Start publishing input velocity
  test_node_->StartPublishVelocity(input_velocity_);

  // Wait for the output to reach the expected value
  EXPECT_TRUE(test_node_->WaitForExpectedVelocity(input_velocity_));

  // verify
  // Ensure input and output match (already verified with WaitForExpectedVelocity, but re-evaluated to record actual values)
  EXPECT_DOUBLE_EQ(input_velocity_.linear.x, test_node_->output_velocity().linear.x);
  EXPECT_DOUBLE_EQ(input_velocity_.linear.y, test_node_->output_velocity().linear.y);
  EXPECT_DOUBLE_EQ(input_velocity_.angular.z, test_node_->output_velocity().angular.z);
}

/// Velocity correction test
/// If impassable, velocity is corrected to stop
TEST_F(BaseVelocityAdjusterNodeTest, StopWhenNoPassableDirection) {
  // setup
  // Turn ON velocity correction function via service
  test_node_->EnableVelocityAdjust();

  // Set obstacles to block the path within a 90° range on both sides
  const double input_direction = atan2(input_velocity_.linear.y, input_velocity_.linear.x);
  PointCloud obstacle;
  for (int32_t i = -static_cast<int32_t>(M_PI / 2.0 / kObstacleDirectionStep);
       i < static_cast<int32_t>(M_PI / 2.0 / kObstacleDirectionStep); ++i) {
    const double obstacle_direction = input_direction + kObstacleDirectionStep * i;
    AddCircularObstacle(cos(obstacle_direction) * kEstimationTime,
                        sin(obstacle_direction) * kEstimationTime,
                        kObstacleRadius, kObstaclePointNum, obstacle);
  }

  // exercise
  // Publish obstacle
  test_node_->PublishObstacle(obstacle);
  // Start publishing input velocity
  test_node_->StartPublishVelocity(input_velocity_);

  // Wait for the output to reach the expected value
  geometry_msgs::msg::Twist expected_velocity;
  expected_velocity.linear.x = 0.0;
  expected_velocity.linear.y = 0.0;
  expected_velocity.angular.z = 0.0;
  EXPECT_TRUE(test_node_->WaitForExpectedVelocity(expected_velocity));

  // verify
  // Ensure output velocity is 0 (already verified with WaitForExpectedVelocity, but re-evaluated to record actual values)
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.x);
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.y);
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().angular.z);
}
}  // namespace tmc_base_velocity_adjuster

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);
  const std::string yaml_directory = ament_index_cpp::get_package_share_directory("tmc_base_velocity_adjuster") +
      "/test/parameter/";
  // Create base_velocity_adjuster node
  auto base_velocity_adjuster_node =
      std::make_shared<tmc_base_velocity_adjuster::BaseVelocityAdjusterNode<
      sensor_msgs::msg::PointCloud2, pcl::PointCloud<pcl::PointXYZ>>>(option);
  // Read parameters from yaml
  tmc_base_velocity_adjuster::LoadParameterFromYaml(
      base_velocity_adjuster_node, yaml_directory, "base_velocity_adjuster_config-test.yaml");
  base_velocity_adjuster_node->Init();
  // Create a thread for spinning
  auto base_velocity_adjuster_node_thread = std::make_shared<std::thread>([&]() {
      rclcpp::spin(base_velocity_adjuster_node);
      });
  testing::InitGoogleTest(&argc, argv);

  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  base_velocity_adjuster_node_thread->join();
  base_velocity_adjuster_node.reset();

  return result;
}
