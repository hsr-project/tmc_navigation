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
/// @file safety_velocity_limiter-test.cpp
/// @brief Test of the safety_velocity_limiter node

#include <stdlib.h>

#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tmc_navigation_msgs/srv/get_current_setting.hpp>
#include <tmc_navigation_msgs/srv/switch_bumper_set.hpp>
#include "../src/param.hpp"
#include "../src/safety_velocity_limiter.hpp"

namespace {
// Point cloud related types
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
// Service name
// Service name for function On
const char* const kStartServiceName = "safety_velocity_limiter/start";
// Service name for function Off
const char* const kStopServiceName = "safety_velocity_limiter/stop";
// Service name for bumper set switching
const char* const kSwitchServiceName = "safety_velocity_limiter/switch_bumper_set";
// Service name to get current settings
const char* const kGetCurrentSettingServiceName = "safety_velocity_limiter/get_current_setting";
// Service name to reset to default settings
const char* const kResetToDefaultServiceName = "safety_velocity_limiter/reset_to_default";

// Topic name
const char* const kTopicObstacleCloud = "obstacle_cloud";    // PointCloud Topic name
const char* const kTopicInputVelocity = "input_velocity";    // Input velocity Topic name
const char* const kTopicOutputVelocity = "output_velocity";  // Output velocity Topic name
const char* const kTopicOccupancy = "obstacle_map";          // Occupancy Grid
const char* const kTopicSlowingDown = "slowing_down";        // Sudden deceleration notification Topic name
const char* const kTopicObservedObstaclePose = "safety_velocity_limiter/observed_obstacle_pose";   // Obstacle coordinates Topic name
const char* const kTopicRatio = "safety_velocity_limiter/ratio";   // Deceleration ratio Topic name
// Topic buffer size
const int32_t kTopicBufferSize = 1;
// Waiting time between tests [msec]
const int32_t kSleepDuration = 100;
// Waiting time for service provision [msec]
const int32_t kWaitDurationForService = 1000;
// Test update cycle [hz]
const double kTestRate = 100.0;
// Input velocity
const double kTestParamVelocity = 2.0;
// Input velocity (high speed)
const double kTestParamFastVelocity = 5.0;
// Communication timeout time [sec]
const double kConnectionTimeOut = 3.0;
// Velocity convergence timeout time [sec]
const double kStabilizationTimeOut = 8.0;
// Velocity convergence timeout time (for high speed) [sec]
const double kStabilizationTimeOutForFastVelocity = 20.0;
// Subscribe timeout time [sec]
const double kSubscribeTimeOut = 1.0;
// Transform timeout time [sec]
const double kTransformTimeOut = 1.0;
// Velocity Publish timeout time [sec]
const double kPublishVelocityTimeOut = 2.0;

// Distance to obstacle: very close
const double kObstacleDistanceClose = 0.3;
// Distance to obstacle: close
const double kObstacleDistanceNear = 0.6;
// Distance to obstacle: intermediate
const double kObstacleDistanceMiddle = 0.8;
// Distance to obstacle: far
const double kObstacleDistanceFar = 1.0;
// Distance to obstacle: very far
const double kObstacleDistanceOutofRange = 2.0;
// Robot radius
const double kRobotRadius = 0.22;
// Map resolution
const double kMapResolution = 0.05;
// Maximum acceleration
const double kMaximumAcceleration = 0.5;
// Maximum deceleration
const double kMaximumDeceleration = 1.0;

// Floating-point error tolerance
const double kEpsilon = 1e-5;

// TODO(syuuhei_shiro): tmc_rostest_utilをROS2化してそこに置く
// Load parameters from yaml file
void LoadParameterFromYaml(std::shared_ptr<rclcpp::Node> node,
    const std::string& yaml_directory, const std::string& yaml_name) {
  const std::string yaml_path = yaml_directory + yaml_name;
  // Load yaml and generate ParameterMap
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_params_t* yaml_params = rcl_yaml_node_struct_init(allocator);
  rcl_parse_yaml_file(yaml_path.c_str(), yaml_params);
  rclcpp::ParameterMap yaml_param_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);
  // Set ros parameters to node
  const std::string parameter_space = "/" + std::string(node->get_name());
  auto iter = yaml_param_map.find(parameter_space);
  for (auto& param : iter->second) {
    node->declare_parameter(param.get_name(), param.get_type());
    node->set_parameter(param);
  }
}

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(const double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

/// Create an OccupancyGrid centered on the specified Point (base_link) with size×size
/// Set all occupancy values to the value specified by default_value
void CreateOccupancyGrid(const geometry_msgs::msg::Point& center,
                         const uint32_t size, const uint32_t default_value,
                         nav_msgs::msg::OccupancyGrid& occupancy_grid) {
  occupancy_grid.header.frame_id = "map";
  occupancy_grid.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  occupancy_grid.info.resolution = kMapResolution;
  occupancy_grid.info.width = size;
  occupancy_grid.info.height = size;
  occupancy_grid.info.origin.position.x = center.x - kMapResolution * static_cast<double>(size) / 2.0;
  occupancy_grid.info.origin.position.y = center.y - kMapResolution * static_cast<double>(size) / 2.0;
  occupancy_grid.info.origin.orientation = createQuaternionMsgFromYaw(0.0);
  occupancy_grid.data.resize(size * size);
  for (uint32_t y = 0; y < size; ++y) {
    for (uint32_t x = 0; x < size; ++x) {
      occupancy_grid.data[y * size + x] = default_value;
    }
  }
}

/// Set the Occupancy value at the specified Point (base_link) in the OccupancyGrid
/// Do nothing if the specified Point is outside the range of the OccupancyGrid
void SetOccupancy(nav_msgs::msg::OccupancyGrid& occupancy_grid,
                  const geometry_msgs::msg::Point& occupancy_point,
                  const uint32_t value) {
  const double offset_x = occupancy_point.x - occupancy_grid.info.origin.position.x;
  const double offset_y = occupancy_point.y - occupancy_grid.info.origin.position.y;
  const int32_t x = static_cast<int32_t>(offset_x / occupancy_grid.info.resolution);
  const int32_t y = static_cast<int32_t>(offset_y / occupancy_grid.info.resolution);
  if (x >= 0 && x < static_cast<int32_t>(occupancy_grid.info.width) &&
      y >= 0 && y < static_cast<int32_t>(occupancy_grid.info.height)) {
    occupancy_grid.data[y * occupancy_grid.info.width + x] = value;
  } else {
    // Do nothing if out of range
  }
}
}  // anonymous namespace

namespace tmc_safety_velocity_limiter {
using std::placeholders::_1;

/// Test node class
class TestNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options), rate_(kTestRate) {}

  /// Initialization
  void Init();

  /// Spin the test node
  void SpinOnce() {
    rclcpp::spin_some(shared_from_this());
  }

  /// Start speed limiting
  void StartLimitVelocity() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result_future = start_service_->async_send_request(request);
    rclcpp::spin_until_future_complete(shared_from_this(), result_future);
  }

  /// Stop speed limiting
  void StopLimitVelocity() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result_future = stop_service_->async_send_request(request);
    rclcpp::spin_until_future_complete(shared_from_this(), result_future);
  }

  /// Switch bumper set
  bool SwitchBumperSet(const std::string& buper_set_name,
      const std::vector<std_msgs::msg::String>& disable_bumpers = std::vector<std_msgs::msg::String>()) {
    auto request = std::make_shared<tmc_navigation_msgs::srv::SwitchBumperSet::Request>();
    request->bumper_set.data = buper_set_name;
    request->disable_bumpers = disable_bumpers;
    auto result_future = switch_service_->async_send_request(request);
    rclcpp::spin_until_future_complete(shared_from_this(), result_future);
    return result_future.get()->is_success;
  }

  /// Get current settings
  tmc_navigation_msgs::srv::GetCurrentSetting::Response GetCurrentSetting() {
    auto request = std::make_shared<tmc_navigation_msgs::srv::GetCurrentSetting::Request>();
    auto result_future = get_current_setting_service_->async_send_request(request);
    rclcpp::spin_until_future_complete(shared_from_this(), result_future);
    return *(result_future.get());
  }

  /// Reset to default settings
  void ResetToDefault() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result_future = reset_to_default_service_->async_send_request(request);
    rclcpp::spin_until_future_complete(shared_from_this(), result_future);
  }

  // Send StaticTF
  void SendStaticTransformMapToBase() {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "base_link";
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation = createQuaternionMsgFromYaw(0.0);
    tf_static_broadcaster_->sendTransform(transform_stamped);
  }

  // Publish PointCloud at the specified Point (base_link)
  void PublishPointCloud(const std::vector<geometry_msgs::msg::Point>& points);

  // Wait for speed to converge due to acceleration limiting function
  bool WaitStabilization(const double timeout, const geometry_msgs::msg::Twist& input_velocity);

  // Publish OccupancyGrid
  void PublishOccupancyGrid(const nav_msgs::msg::OccupancyGrid& occupancy_grid);
  // Publish velocity command value
  void PublishVelocity(const geometry_msgs::msg::Twist& velocity);
  /// Publish speed for the specified time
  bool PublishVelocityForSpecifiedTime(const geometry_msgs::msg::Twist& velocity, const double sec);

  // Wait for topic subscription
  // Wait for velocity topic subscription
  bool WaitForSubscribeVelocity(const double timeout);
  // Wait for deceleration notification topic subscription
  bool WaitForSubscribeSlowdown(const double timeout);
  // Wait for limiting factor Pose topic subscription
  bool WaitForSubscribeObservedObstaclePose(const double timeout);
  // Function to wait for the startup of the subscriber under test
  bool WaitForConnectionEstablished(const double timeout);

  // getter
  geometry_msgs::msg::Twist output_velocity() const { return output_velocity_; }
  double ratio() const { return ratio_; }
  geometry_msgs::msg::PoseStamped observed_obstacle_pose() const { return observed_obstacle_pose_; }
  bool is_slowing_down() const { return is_slowing_down_; }

 private:
  // Output velocity callback function
  void OutputVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // slowing_down topic subscription callback function
  void SlowingDownCallback(const std_msgs::msg::Bool::SharedPtr msg);
  // Limiting factor Pose topic subscription callback function
  void ObservedObstaclePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  // ratio topic subscription callback function
  void RatioCallback(const std_msgs::msg::Float64::SharedPtr msg);

  // Cycle weight
  rclcpp::Rate rate_;

  // Service client
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_service_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_service_;
  rclcpp::Client<tmc_navigation_msgs::srv::SwitchBumperSet>::SharedPtr switch_service_;
  rclcpp::Client<tmc_navigation_msgs::srv::GetCurrentSetting>::SharedPtr get_current_setting_service_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_to_default_service_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_cloud_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_input_velocity_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_output_velocity_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_slowing_down_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_observed_obstacle_pose_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_ratio_;

  // Output velocity to subscribe
  geometry_msgs::msg::Twist output_velocity_;

  // Sudden deceleration detection
  bool is_slowing_down_;

  // Limiting factor Pose
  geometry_msgs::msg::PoseStamped observed_obstacle_pose_;

  // Deceleration ratio
  double ratio_;

  // Subscription flag
  bool velocity_subscribed_;
  bool slowing_down_subscribed_;
  bool observed_obstacle_pose_subscribed_;
  bool ratio_subscribed_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

/// Initialization
void TestNode::Init() {
  tf_buffer_ =
    std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Declaration of service client
  start_service_ = this->create_client<std_srvs::srv::Empty>(kStartServiceName);
  stop_service_ = this->create_client<std_srvs::srv::Empty>(kStopServiceName);
  switch_service_ = this->create_client<tmc_navigation_msgs::srv::SwitchBumperSet>(kSwitchServiceName);
  get_current_setting_service_ = this->create_client<tmc_navigation_msgs::srv::GetCurrentSetting>(
      kGetCurrentSettingServiceName);
  reset_to_default_service_ = this->create_client<std_srvs::srv::Empty>(kResetToDefaultServiceName);

  start_service_->wait_for_service(std::chrono::milliseconds(kWaitDurationForService));
  stop_service_->wait_for_service(std::chrono::milliseconds(kWaitDurationForService));
  switch_service_->wait_for_service(std::chrono::milliseconds(kWaitDurationForService));
  get_current_setting_service_->wait_for_service(std::chrono::milliseconds(kWaitDurationForService));
  reset_to_default_service_->wait_for_service(std::chrono::milliseconds(kWaitDurationForService));

  // Declaration of publisher
  pub_obstacle_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(kTopicObstacleCloud, kTopicBufferSize);
  pub_input_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>(kTopicInputVelocity, kTopicBufferSize);
  pub_occupancy_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(kTopicOccupancy, kTopicBufferSize);

  // Declaration of subscriber
  sub_output_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
      kTopicOutputVelocity, kTopicBufferSize,
      std::bind(&TestNode::OutputVelocityCallback, this, _1));

  sub_slowing_down_ = this->create_subscription<std_msgs::msg::Bool>(
      kTopicSlowingDown, kTopicBufferSize,
      std::bind(&TestNode::SlowingDownCallback, this, _1));
  sub_observed_obstacle_pose_ =  this->create_subscription<geometry_msgs::msg::PoseStamped>(
      kTopicObservedObstaclePose, kTopicBufferSize,
      std::bind(&TestNode::ObservedObstaclePoseCallback, this, _1));
  sub_ratio_ = this->create_subscription<std_msgs::msg::Float64>(
      kTopicRatio, kTopicBufferSize,
      std::bind(&TestNode::RatioCallback, this, _1));

  // Initialize subscription flag
  velocity_subscribed_ = false;
  slowing_down_subscribed_ = false;
  observed_obstacle_pose_subscribed_ = false;
  ratio_subscribed_ = false;
  // Whether it is connected to the target node
  ASSERT_TRUE(WaitForConnectionEstablished(kConnectionTimeOut));
  // Wait for the TF of map->base_link to be published
  SendStaticTransformMapToBase();
  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(kTransformTimeOut);
  ASSERT_TRUE(tf_buffer_->canTransform("map", "base_link", rclcpp::Time(0), timeout));
}

// Publish PointCloud at the specified Point (base_link)
void TestNode::PublishPointCloud(const std::vector<geometry_msgs::msg::Point>& points) {
  // PointCloud data of obstacles
  PointCloud primitive_obstacle_cloud;
  Point cloud_point;
  for (std::vector<geometry_msgs::msg::Point>::const_iterator it = points.begin(); it != points.end(); ++it) {
    cloud_point.x = it->x;
    cloud_point.y = it->y;
    primitive_obstacle_cloud.push_back(cloud_point);
  }
  // PointCloud2 data of obstacles to be published
  sensor_msgs::msg::PointCloud2 obstacle_cloud;
  pcl::toROSMsg(primitive_obstacle_cloud, obstacle_cloud);
  obstacle_cloud.header.frame_id = "map";
  obstacle_cloud.header.stamp = this->get_clock()->now();
  // Publish test data
  pub_obstacle_cloud_->publish(obstacle_cloud);
  SpinOnce();
  rate_.sleep();
}

// Publish OccupancyGrid
void TestNode::PublishOccupancyGrid(const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
  pub_occupancy_->publish(occupancy_grid);
  SpinOnce();
  rate_.sleep();
}

void TestNode::PublishVelocity(const geometry_msgs::msg::Twist& velocity) {
  // Clear subscription flag
  velocity_subscribed_ = false;
  slowing_down_subscribed_ = false;
  observed_obstacle_pose_subscribed_ = false;
  ratio_subscribed_ = false;
  pub_input_velocity_->publish(velocity);
  SpinOnce();
  rate_.sleep();
}

/// Publish speed for the specified time
bool TestNode::PublishVelocityForSpecifiedTime(
    const geometry_msgs::msg::Twist& velocity, const double sec) {
  const rclcpp::Time start_time = this->get_clock()->now();
  while (rclcpp::ok()) {
    PublishVelocity(velocity);
    // End after the specified time has elapsed
    const double elapsed_time = (this->get_clock()->now() - start_time).seconds();
    if (elapsed_time > sec) {
      break;
    }
  }
  return true;
}

/// Callback function for output_velocity
void TestNode::OutputVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  output_velocity_ = *msg;
  velocity_subscribed_ = true;
}

/// Callback function for slowing_down
void TestNode::SlowingDownCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  is_slowing_down_ = msg->data;
  slowing_down_subscribed_ = true;
}

/// Callback function for limiting factor Pose
void TestNode::ObservedObstaclePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  observed_obstacle_pose_ = *msg;
  observed_obstacle_pose_subscribed_ = true;
}

/// Callback function for ratio
void TestNode::RatioCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  ratio_ = msg->data;
  ratio_subscribed_ = true;
}

/// Wait until the connection with the target node is established
bool TestNode::WaitForConnectionEstablished(const double timeout) {
  const rclcpp::Time start_time = this->get_clock()->now();
  while (sub_output_velocity_->get_publisher_count() == 0 ||
         sub_slowing_down_->get_publisher_count() == 0 ||
         sub_observed_obstacle_pose_->get_publisher_count() == 0 ||
         sub_ratio_->get_publisher_count() == 0) {
    if ((this->get_clock()->now() - start_time).seconds() > timeout) {
      return false;
    }
    SpinOnce();
    rate_.sleep();
  }
  return true;
}

/// Wait for speed to converge due to acceleration limiting function
bool TestNode::WaitStabilization(const double timeout, const geometry_msgs::msg::Twist& input_velocity) {
  const rclcpp::Time start_time = this->get_clock()->now();
  bool stabilized = false;
  while (!stabilized && rclcpp::ok()) {
    geometry_msgs::msg::Twist previous_velocity = output_velocity_;
    // Failure if timed out
    const double elapsed_time = (this->get_clock()->now() - start_time).seconds();
    if (elapsed_time > timeout) {
      break;
    }
    PublishVelocity(input_velocity);
    // Subscribe to output velocity
    if (!WaitForSubscribeVelocity(kSubscribeTimeOut)) {
      break;
    }
    // End if there is no change from the previous value
    if (fabs(previous_velocity.linear.x - output_velocity_.linear.x) < kEpsilon &&
        fabs(previous_velocity.linear.y - output_velocity_.linear.y) < kEpsilon) {
      stabilized = true;
      break;
    }
  }
  return stabilized;
}

/// Wait for velocity topic subscription
bool TestNode::WaitForSubscribeVelocity(const double timeout) {
  const rclcpp::Time start_time = this->get_clock()->now();
  while (!velocity_subscribed_ && rclcpp::ok()) {
    // Failure if timed out
    const double elapsed_time = (this->get_clock()->now() - start_time).seconds();
    if (elapsed_time > timeout) {
      break;
    }
    SpinOnce();
    rate_.sleep();
  }
  return velocity_subscribed_;
}

/// Wait for deceleration notification topic subscription
bool TestNode::WaitForSubscribeSlowdown(const double timeout) {
  const rclcpp::Time start_time = this->get_clock()->now();
  while (!slowing_down_subscribed_ && rclcpp::ok()) {
    // Failure if timed out
    const double elapsed_time = (this->get_clock()->now() - start_time).seconds();
    if (elapsed_time > timeout) {
      break;
    }
    SpinOnce();
    rate_.sleep();
  }
  return slowing_down_subscribed_;
}

/// Wait for limiting factor Pose topic subscription
bool TestNode::WaitForSubscribeObservedObstaclePose(const double timeout) {
  const rclcpp::Time start_time = this->get_clock()->now();
  while (!observed_obstacle_pose_subscribed_ && rclcpp::ok()) {
    // Failure if timed out
    const double elapsed_time = (this->get_clock()->now() - start_time).seconds();
    if (elapsed_time > timeout) {
      break;
    }
    SpinOnce();
    rate_.sleep();
  }
  return observed_obstacle_pose_subscribed_;
}

/// Test fixture
class SafetyVelocityLimiterNodeTest : public testing::Test {
 public:
  SafetyVelocityLimiterNodeTest() {}
  virtual ~SafetyVelocityLimiterNodeTest() = default;

 protected:
  virtual void SetUp() {
    // Generate test node
    test_node_ = std::make_shared<TestNode>();
    test_node_->Init();
    // Set input velocity
    input_velocity_.linear.x = kTestParamVelocity;
    input_velocity_.linear.y = 0.0;
    input_velocity_.linear.z = 0.0;
    input_velocity_.angular.x = 0.0;
    input_velocity_.angular.y = 0.0;
    input_velocity_.angular.z = 0.0;
  }
  virtual void TearDown() {
    // Publish an empty point cloud
    std::vector<geometry_msgs::msg::Point> empty_point;
    test_node_->PublishPointCloud(empty_point);
  }

  // Test node
  std::shared_ptr<TestNode> test_node_;
  // Output velocity to be published
  geometry_msgs::msg::Twist input_velocity_;
};

/// Confirm whether the stop process is effective when the speed limiting function is On
TEST_F(SafetyVelocityLimiterNodeTest, StopAgainstObstacle) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to fan-shaped bumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("triangle_bumper_test"));
  // Publish the point cloud coordinates of obstacles within the stop threshold distance from the vehicle and input velocity
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = kObstacleDistanceClose;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));

  // Confirm whether the error between the actual output velocity and the target velocity (0 in this pattern) is within the allowable error
  // If within the allowable error, determine that the stop has been executed
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.x);
}

/// Confirm whether the deceleration process according to the distance between the obstacle and the vehicle is effective when the speed limiting function is On
TEST_F(SafetyVelocityLimiterNodeTest, ReduceVelocityAgainstObstacle) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to fan-shaped bumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("triangle_bumper_test"));

  // Place an obstacle within the speed limiting range
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = kObstacleDistanceNear;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Store the output velocity subscribed under the above conditions in output_velocity1
  geometry_msgs::msg::Twist output_velocity1 = test_node_->output_velocity();

  // Place an obstacle far away
  obstacle_point_on_base[0].x = kObstacleDistanceFar;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Store the output velocity subscribed under the above conditions in output_velocity2
  geometry_msgs::msg::Twist output_velocity2 = test_node_->output_velocity();

  // Confirm that the output velocity is (when the obstacle is close < when the obstacle is far)
  EXPECT_LT(output_velocity1.linear.x, output_velocity2.linear.x);
}

/// Confirm whether the stop process is ineffective when the obstacle is outside the speed limiting range even if the speed limiting function is On
TEST_F(SafetyVelocityLimiterNodeTest, OutOfVelocityLimitation) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to fan-shaped bumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("triangle_bumper_test"));
  // Place an obstacle outside the stop range
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = kObstacleDistanceOutofRange;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is not restricted
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);
}

/// Confirm whether the deceleration process according to the distance between multiple obstacles and the vehicle is effective when the speed limiting function is On
TEST_F(SafetyVelocityLimiterNodeTest, ReduceVelocityAgainstNearestObstacles) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to fan-shaped bumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("triangle_bumper_test"));

  // Place one obstacle within the speed limiting range
  geometry_msgs::msg::Point point;
  std::vector<geometry_msgs::msg::Point> multi_obstacle_points_on_base;
  point.x = kObstacleDistanceMiddle;
  multi_obstacle_points_on_base.push_back(point);
  test_node_->PublishPointCloud(multi_obstacle_points_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Store the output velocity subscribed under the above conditions in output_velocity1
  geometry_msgs::msg::Twist output_velocity1 = test_node_->output_velocity();
  // Confirm that the deceleration ratio is published correctly
  EXPECT_DOUBLE_EQ(input_velocity_.linear.x * test_node_->ratio(), test_node_->output_velocity().linear.x);

  // Place an additional obstacle within the speed limiting range in addition to the above obstacle
  // However, the closest obstacle to the vehicle is the one that was originally there
  point.x = kObstacleDistanceFar;
  multi_obstacle_points_on_base.push_back(point);
  test_node_->PublishPointCloud(multi_obstacle_points_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Store the output velocity subscribed under the above conditions in output_velocity2
  geometry_msgs::msg::Twist output_velocity2 = test_node_->output_velocity();
  // Confirm that the output velocity does not change whether the obstacle is single or multiple if the closest obstacle to the vehicle is the same
  EXPECT_DOUBLE_EQ(output_velocity1.linear.x, output_velocity2.linear.x);
  // Confirm that the deceleration ratio is published correctly
  EXPECT_DOUBLE_EQ(input_velocity_.linear.x * test_node_->ratio(), test_node_->output_velocity().linear.x);

  // Place an additional obstacle within the speed limiting range in addition to the above obstacle
  // However, the closest obstacle to the vehicle is the newly added obstacle
  point.x = kObstacleDistanceNear;
  multi_obstacle_points_on_base.push_back(point);
  test_node_->PublishPointCloud(multi_obstacle_points_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Store the output velocity subscribed under the above conditions in output_velocity3
  geometry_msgs::msg::Twist output_velocity3 = test_node_->output_velocity();
  // Confirm that the deceleration ratio is published correctly
  EXPECT_DOUBLE_EQ(input_velocity_.linear.x * test_node_->ratio(), test_node_->output_velocity().linear.x);

  // Confirm that the output velocity decreases when the closest obstacle to the vehicle becomes closer
  EXPECT_LT(output_velocity3.linear.x, output_velocity1.linear.x);
}

/// Confirm whether the stop process according to the distance between multiple obstacles and the vehicle is effective when the speed limiting function is On
TEST_F(SafetyVelocityLimiterNodeTest, StopAgainstMultiObstacles) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to fan-shaped bumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("triangle_bumper_test"));

  // Place multiple obstacles within the speed limiting range
  // However, the closest obstacle to the vehicle is within the stop threshold distance
  std::vector<geometry_msgs::msg::Point> multi_obstacle_points_on_base(3, geometry_msgs::msg::Point());
  multi_obstacle_points_on_base[0].x = kObstacleDistanceMiddle;
  multi_obstacle_points_on_base[1].x = kObstacleDistanceNear;
  multi_obstacle_points_on_base[2].x = kObstacleDistanceClose;
  test_node_->PublishPointCloud(multi_obstacle_points_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Store the output velocity subscribed under the above conditions in output_velocity2
  geometry_msgs::msg::Twist output_velocity = test_node_->output_velocity();

  // Confirm whether the error between the actual output velocity and the target velocity (0 in this pattern) is within the allowable error
  // If within the allowable error, determine that the stop has been executed
  EXPECT_DOUBLE_EQ(0.0, output_velocity.linear.x);
}

/// Confirm whether the stop process is ineffective when the speed limiting function is Off
TEST_F(SafetyVelocityLimiterNodeTest, OffStopAgainstObstacle) {
  // First, confirm that the stop process is effective when the speed limiting function is On
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to fan-shaped bumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("triangle_bumper_test"));

  // Place an obstacle within the stop range
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = kObstacleDistanceClose;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is restricted (stopped)
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.x);

  // Next, confirm that the stop process is ineffective when the speed limiting function is Off
  // Disable speed limiting
  test_node_->StopLimitVelocity();
  // Place an obstacle within the stop range
  obstacle_point_on_base[0].x = kObstacleDistanceClose;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is not restricted
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);
}

/// Triangle bumper
TEST_F(SafetyVelocityLimiterNodeTest, TriangleBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to TriangleBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("triangle_bumper_test"));

  // Publish an obstacle outside the left front bumper range (TriangleBumper's blind spot)
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = kRobotRadius;
  obstacle_point_on_base[0].y = kRobotRadius;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is not restricted
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);
  // Publish an obstacle within the left front bumper range
  obstacle_point_on_base[0].x = kRobotRadius;
  obstacle_point_on_base[0].y = kRobotRadius - 0.1;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is restricted
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.x);

  // Publish an obstacle in front of the vehicle
  obstacle_point_on_base[0].x = kRobotRadius;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is restricted
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.x);

  // Publish an obstacle within the right front bumper range
  obstacle_point_on_base[0].x = kRobotRadius;
  obstacle_point_on_base[0].y = -kRobotRadius + 0.1;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is restricted
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.x);

  // Publish an obstacle outside the right front bumper range (TriangleBumper's blind spot)
  obstacle_point_on_base[0].x = kRobotRadius;
  obstacle_point_on_base[0].y = -kRobotRadius;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is not restricted
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);
}

/// Cup bumper
TEST_F(SafetyVelocityLimiterNodeTest, CupBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to CupBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("cup_bumper_test"));

  // Publish an obstacle within the rear bumper range of the vehicle
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = -kRobotRadius;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is restricted
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.x);

  // Publish an obstacle outside the rear bumper range of the vehicle
  obstacle_point_on_base[0].x = -kRobotRadius - 0.1;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is not restricted
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);

  // Publish an obstacle outside the left side bumper range of the vehicle
  obstacle_point_on_base[0].x = 0.0;
  obstacle_point_on_base[0].y = kObstacleDistanceClose;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is not restricted
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);

  // Publish an obstacle within the left side bumper range of the vehicle
  obstacle_point_on_base[0].x = 0.0;
  obstacle_point_on_base[0].y = kObstacleDistanceClose - 0.1;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is restricted
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.x);

  // Publish an obstacle within the right side bumper range of the vehicle
  obstacle_point_on_base[0].x = 0.0;
  obstacle_point_on_base[0].y = -kObstacleDistanceClose + 0.1;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is restricted
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.x);

  // Publish an obstacle outside the right side bumper range of the vehicle
  obstacle_point_on_base[0].x = 0.0;
  obstacle_point_on_base[0].y = -kObstacleDistanceClose;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is not restricted
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);
}

/// Ellipse bumper
TEST_F(SafetyVelocityLimiterNodeTest, EllipseBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to EllipseBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("ellipse_bumper_test"));

  // Publish an obstacle within the rear bumper range of the vehicle
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = -kRobotRadius;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is restricted
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.x);

  // Publish an obstacle outside the rear bumper range of the vehicle
  obstacle_point_on_base[0].x = -kObstacleDistanceClose;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is not restricted
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);

  // Publish an obstacle within the right front bumper range of the vehicle
  obstacle_point_on_base[0].x = kObstacleDistanceClose;
  obstacle_point_on_base[0].y = -kObstacleDistanceClose;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is restricted
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.x);

  // Publish an obstacle outside the right front bumper range of the vehicle
  obstacle_point_on_base[0].x = kObstacleDistanceClose;
  obstacle_point_on_base[0].y = -kObstacleDistanceClose - 0.2;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is not restricted
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);

  // Publish an obstacle within the stop area range in front of the vehicle
  obstacle_point_on_base[0].x = kObstacleDistanceClose;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is restricted
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.x);

  // Publish an obstacle within the deceleration area range in front of the vehicle
  obstacle_point_on_base[0].x = kObstacleDistanceFar;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is restricted
  EXPECT_GT(kTestParamVelocity, test_node_->output_velocity().linear.x);

  // Publish an obstacle outside the bumper range in front of the vehicle
  obstacle_point_on_base[0].x = kObstacleDistanceOutofRange;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  // Wait for output to converge
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that it is not restricted
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);
}

/// OccupancyPoint bumper
TEST_F(SafetyVelocityLimiterNodeTest, OccupancyPointBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to OccupancyPointBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("occupancy_point_bumper_test"));

  nav_msgs::msg::OccupancyGrid occupancy_grid;
  // Publish an OccupancyGrid with Occupancy0 around the vehicle's position
  CreateOccupancyGrid(geometry_msgs::msg::Point(), 10, 0, occupancy_grid);
  test_node_->PublishOccupancyGrid(occupancy_grid);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity1 = test_node_->output_velocity();

  // Set and publish Occupancy100 at the vehicle's position
  SetOccupancy(occupancy_grid, geometry_msgs::msg::Point(), 100);
  test_node_->PublishOccupancyGrid(occupancy_grid);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity2 = test_node_->output_velocity();

  // Confirm that it is not restricted with Occupancy0
  EXPECT_DOUBLE_EQ(kTestParamVelocity, output_velocity1.linear.x);
  // Confirm that it is stopped with Occupancy100
  EXPECT_DOUBLE_EQ(0.0, output_velocity2.linear.x);
}

/// OccupancyEllipse bumper
TEST_F(SafetyVelocityLimiterNodeTest, OccupancyEllipseBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to OccupancyEllipseBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("occupancy_ellipse_bumper_test"));

  // Generate a 10×10 OccupancyGrid with values set from 0 to 99
  nav_msgs::msg::OccupancyGrid gradation_occupancy;
  const uint32_t size = 10;
  CreateOccupancyGrid(geometry_msgs::msg::Point(), size, 0, gradation_occupancy);
  for (uint32_t y = 0; y < size; ++y) {
    for (uint32_t x = 0; x < size; ++x) {
      gradation_occupancy.data[y * gradation_occupancy.info.width + x] = y * gradation_occupancy.info.width + x;
    }
  }
  // Set the terminal point to 100
  gradation_occupancy.data[size * size - 1] = 100;

  // Set to a position where the Occupancy around the vehicle's position is low
  gradation_occupancy.info.origin.position.x = 0.0;
  gradation_occupancy.info.origin.position.y = 0.0;
  test_node_->PublishOccupancyGrid(gradation_occupancy);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity1 = test_node_->output_velocity();

  // Set to a position where the Occupancy around the vehicle's position is high
  gradation_occupancy.info.origin.position.x = -0.45;
  gradation_occupancy.info.origin.position.y = -0.45;
  test_node_->PublishOccupancyGrid(gradation_occupancy);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity2 = test_node_->output_velocity();

  // Confirm that it is not at extreme values since a weighted average is taken in a circular range
  EXPECT_LT(0.0, output_velocity1.linear.x);
  EXPECT_LT(output_velocity2.linear.x, kTestParamVelocity);
  // Confirm that it is slower when the surrounding Occupancy is high
  EXPECT_LT(output_velocity2.linear.x, output_velocity1.linear.x);

  // Set Occupancy value 100 outside the bumper range
  nav_msgs::msg::OccupancyGrid out_range_occupancy;
  CreateOccupancyGrid(geometry_msgs::msg::Point(), 25, 0, out_range_occupancy);
  geometry_msgs::msg::Point occupancy_point;
  occupancy_point.x = 1.0;
  occupancy_point.y = 1.0;
  SetOccupancy(out_range_occupancy, occupancy_point, 100);
  test_node_->PublishOccupancyGrid(out_range_occupancy);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity3 = test_node_->output_velocity();
  // Confirm that it is not affected by Occupancy existing outside the bumper range
  EXPECT_DOUBLE_EQ(kTestParamVelocity, output_velocity3.linear.x);

  // Set Occupancy value 100 within the bumper range
  nav_msgs::msg::OccupancyGrid in_range_occupancy;
  CreateOccupancyGrid(geometry_msgs::msg::Point(), 25, 0, in_range_occupancy);
  occupancy_point.x = 0.5;
  occupancy_point.y = 0.5;
  SetOccupancy(in_range_occupancy, occupancy_point, 100);
  test_node_->PublishOccupancyGrid(in_range_occupancy);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity4 = test_node_->output_velocity();
  // Confirm that restriction is applied when there is one point of Occupancy within the bumper range
  EXPECT_LT(output_velocity4.linear.x, kTestParamVelocity);
}

/// Fixed slope
TEST_F(SafetyVelocityLimiterNodeTest, FixedSlope) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to bumper using FixedSlope
  ASSERT_TRUE(test_node_->SwitchBumperSet("fixed_slope_test"));

  // Place an obstacle far away within the speed limiting range
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = kObstacleDistanceFar;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity1 = test_node_->output_velocity();
  // Place an obstacle at an intermediate distance within the speed limiting range
  obstacle_point_on_base[0].x = kObstacleDistanceMiddle;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity2 = test_node_->output_velocity();
  // Place an obstacle nearby within the speed limiting range
  obstacle_point_on_base[0].x = kObstacleDistanceNear;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity3 = test_node_->output_velocity();
  // Confirm that restriction is being applied
  EXPECT_NEAR(output_velocity3.linear.x, 0.2 * kTestParamVelocity, kEpsilon);
  // Confirm that it is the same value regardless of distance
  EXPECT_DOUBLE_EQ(output_velocity1.linear.x, output_velocity2.linear.x);
  EXPECT_DOUBLE_EQ(output_velocity2.linear.x, output_velocity3.linear.x);
}

/// Linear slope
TEST_F(SafetyVelocityLimiterNodeTest, LinearSlope) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to bumper using FixedSlope
  ASSERT_TRUE(test_node_->SwitchBumperSet("linear_slope_test"));

  // Place an obstacle far away within the speed limiting range
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = kObstacleDistanceFar;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity1 = test_node_->output_velocity();
  // Place an obstacle slightly far away within the speed limiting range
  obstacle_point_on_base[0].x = (kObstacleDistanceFar + kObstacleDistanceMiddle)/2;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity2 = test_node_->output_velocity();
  // Place an obstacle at an intermediate distance within the speed limiting range
  obstacle_point_on_base[0].x = kObstacleDistanceMiddle;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity3 = test_node_->output_velocity();
  // Place an obstacle slightly nearby within the speed limiting range
  obstacle_point_on_base[0].x = (kObstacleDistanceMiddle + kObstacleDistanceNear)/2;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity4 = test_node_->output_velocity();
  // Place an obstacle nearby within the speed limiting range
  obstacle_point_on_base[0].x = kObstacleDistanceNear;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity5 = test_node_->output_velocity();
  // Confirm that the speed is in the order of nearby < slightly nearby < intermediate < slightly far away < far away
  EXPECT_LT(output_velocity5.linear.x, output_velocity4.linear.x);
  EXPECT_LT(output_velocity4.linear.x, output_velocity3.linear.x);
  EXPECT_LT(output_velocity3.linear.x, output_velocity2.linear.x);
  EXPECT_LT(output_velocity2.linear.x, output_velocity1.linear.x);
  // Confirm that the slope is linear
  EXPECT_NEAR(output_velocity5.linear.x - output_velocity4.linear.x,
              output_velocity4.linear.x - output_velocity3.linear.x,
              kEpsilon);
  EXPECT_NEAR(output_velocity4.linear.x - output_velocity3.linear.x,
              output_velocity3.linear.x - output_velocity2.linear.x,
              kEpsilon);
  EXPECT_NEAR(output_velocity3.linear.x - output_velocity2.linear.x,
              output_velocity2.linear.x - output_velocity1.linear.x,
              kEpsilon);
}

/// Logarithm slope
TEST_F(SafetyVelocityLimiterNodeTest, LogarithmSlope) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to bumper using FixedSlope
  ASSERT_TRUE(test_node_->SwitchBumperSet("logarithm_slope_test"));

  // Place an obstacle far away within the speed limiting range
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = kObstacleDistanceFar;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity1 = test_node_->output_velocity();
  // Place an obstacle slightly far away within the speed limiting range
  obstacle_point_on_base[0].x = (kObstacleDistanceFar + kObstacleDistanceMiddle)/2;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity2 = test_node_->output_velocity();
  // Place an obstacle at an intermediate distance within the speed limiting range
  obstacle_point_on_base[0].x = kObstacleDistanceMiddle;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity3 = test_node_->output_velocity();
  // Place an obstacle slightly nearby within the speed limiting range
  obstacle_point_on_base[0].x = (kObstacleDistanceMiddle + kObstacleDistanceNear)/2;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity4 = test_node_->output_velocity();
  // Place an obstacle nearby within the speed limiting range
  obstacle_point_on_base[0].x = kObstacleDistanceNear;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  geometry_msgs::msg::Twist output_velocity5 = test_node_->output_velocity();
  // Confirm that the speed is in the order of nearby < slightly nearby < intermediate < slightly far away < far away
  EXPECT_LT(output_velocity5.linear.x, output_velocity4.linear.x);
  EXPECT_LT(output_velocity4.linear.x, output_velocity3.linear.x);
  EXPECT_LT(output_velocity3.linear.x, output_velocity2.linear.x);
  EXPECT_LT(output_velocity2.linear.x, output_velocity1.linear.x);
  // Confirm that the slope is logarithmic
  EXPECT_LT(output_velocity1.linear.x - output_velocity2.linear.x,
            output_velocity2.linear.x - output_velocity3.linear.x);
  EXPECT_LT(output_velocity2.linear.x - output_velocity3.linear.x,
            output_velocity3.linear.x - output_velocity4.linear.x);
  EXPECT_LT(output_velocity3.linear.x - output_velocity4.linear.x,
            output_velocity4.linear.x - output_velocity5.linear.x);
}

/// Sudden deceleration detection
TEST_F(SafetyVelocityLimiterNodeTest, Slowdown) {
  // Turn off the function once to reset sudden deceleration detection
  test_node_->StopLimitVelocity();
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to TriangleBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("triangle_bumper_test"));

  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  // Gradually bring obstacles closer within the deceleration area
  for (double distance = 1.5; distance > 1.0; distance -= 0.1) {
    obstacle_point_on_base[0].x = distance;
    test_node_->PublishPointCloud(obstacle_point_on_base);
    test_node_->PublishVelocity(input_velocity_);
  }
  // Confirm that sudden deceleration is not detected
  EXPECT_FALSE(test_node_->is_slowing_down());

  // Remove the obstacle once and recover to constant speed
  obstacle_point_on_base[0].x = 10.0;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);

  // Place an obstacle in the stop area
  obstacle_point_on_base[0].x = kObstacleDistanceClose;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeSlowdown(kSubscribeTimeOut));
  // Confirm that sudden deceleration is detected
  EXPECT_TRUE(test_node_->is_slowing_down());
}

/// Deceleration acceleration limit
TEST_F(SafetyVelocityLimiterNodeTest, ModerateDeceleration) {
  // Increase input velocity to observe the slope of velocity change
  input_velocity_.linear.x = kTestParamFastVelocity;
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to TriangleBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("triangle_bumper_test"));

  // Set to a state where there are no obstacles within the bumper range and wait for the speed to match the input
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = 10.0;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOutForFastVelocity, input_velocity_));
  EXPECT_DOUBLE_EQ(kTestParamFastVelocity, test_node_->output_velocity().linear.x);

  // Place an obstacle in the deceleration area
  obstacle_point_on_base[0].x = 0.5;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);

  // Input and output speed
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  rclcpp::Time velocity1_time = test_node_->get_clock()->now();
  geometry_msgs::msg::Twist velocity1 = test_node_->output_velocity();

  // Publish speed for the specified time
  test_node_->PublishVelocityForSpecifiedTime(input_velocity_, kPublishVelocityTimeOut);
  // Input and output speed
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  rclcpp::Time velocity2_time = test_node_->get_clock()->now();
  geometry_msgs::msg::Twist velocity2 = test_node_->output_velocity();

  // Confirm that it is decelerating from velocity1 to velocity2
  EXPECT_LT(velocity2.linear.x, velocity1.linear.x);
  // Calculate the absolute value of the slope from velocity1 to velocity2
  double time_diff = (velocity2_time - velocity1_time).seconds();
  double velocity_diff = fabs(velocity1.linear.x - velocity2.linear.x);
  const double velocity_slope_1_2 = velocity_diff / time_diff;
  // Confirm that the slope approximates the expected value kMaximumDeceleration (less than 10% difference)
  EXPECT_LT(fabs((velocity_slope_1_2 - kMaximumDeceleration) / kMaximumDeceleration), 0.1);

  // Publish speed for the specified time
  test_node_->PublishVelocityForSpecifiedTime(input_velocity_, kPublishVelocityTimeOut);
  // Input and output speed
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  rclcpp::Time velocity3_time = test_node_->get_clock()->now();
  geometry_msgs::msg::Twist velocity3 = test_node_->output_velocity();

  // Confirm that it is decelerating from velocity2 to velocity3
  EXPECT_LT(velocity3.linear.x, velocity2.linear.x);
  // Calculate the absolute value of the slope from velocity2 to velocity3
  time_diff = (velocity3_time - velocity2_time).seconds();
  velocity_diff = fabs(velocity2.linear.x - velocity3.linear.x);
  const double velocity_slope_2_3 = velocity_diff / time_diff;
  // Confirm that the slope approximates the expected value kMaximumDeceleration (less than 10% difference)
  EXPECT_LT(fabs((velocity_slope_2_3 - kMaximumDeceleration) / kMaximumDeceleration), 0.1);

  // Confirm that it is decelerating linearly. The two slopes are approximate (less than 10% difference)
  EXPECT_LT(fabs((velocity_slope_2_3 - velocity_slope_1_2) / velocity_slope_1_2), 0.1);
}

/// Acceleration limit during acceleration
TEST_F(SafetyVelocityLimiterNodeTest, ModerateAcceleration) {
  // Increase input velocity to observe the slope of velocity change
  input_velocity_.linear.x = kTestParamFastVelocity;
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to TriangleBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("triangle_bumper_test"));

  // Place an obstacle in the deceleration area and wait for the speed to converge
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = 0.5;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOutForFastVelocity, input_velocity_));

  // Set to a state where there are no obstacles within the bumper range
  obstacle_point_on_base[0].x = 10.0;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);

  // Input and output speed
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  rclcpp::Time velocity1_time = test_node_->get_clock()->now();
  geometry_msgs::msg::Twist velocity1 = test_node_->output_velocity();

  // Publish speed for the specified time
  test_node_->PublishVelocityForSpecifiedTime(input_velocity_, kPublishVelocityTimeOut);
  // Input and output speed
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  rclcpp::Time velocity2_time = test_node_->get_clock()->now();
  geometry_msgs::msg::Twist velocity2 = test_node_->output_velocity();

  // Confirm that it is accelerating from velocity1 to velocity2
  EXPECT_GT(velocity2.linear.x, velocity1.linear.x);
  // Calculate the absolute value of the slope from velocity1 to velocity2
  double time_diff = (velocity2_time - velocity1_time).seconds();
  double velocity_diff = fabs(velocity1.linear.x - velocity2.linear.x);
  const double velocity_slope_1_2 = velocity_diff / time_diff;
  // Confirm that the slope approximates the expected value kMaximumAcceleration (less than 10% difference)
  EXPECT_LT(fabs((velocity_slope_1_2 - kMaximumAcceleration) / kMaximumAcceleration), 0.1);

  // Publish speed for the specified time
  test_node_->PublishVelocityForSpecifiedTime(input_velocity_, kPublishVelocityTimeOut);
  // Input and output speed
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  rclcpp::Time velocity3_time = test_node_->get_clock()->now();
  geometry_msgs::msg::Twist velocity3 = test_node_->output_velocity();

  // Confirm that it is accelerating from velocity2 to velocity3
  EXPECT_GT(velocity3.linear.x, velocity2.linear.x);
  // Calculate the absolute value of the slope from velocity2 to velocity3
  time_diff = (velocity3_time - velocity2_time).seconds();
  velocity_diff = fabs(velocity2.linear.x - velocity3.linear.x);
  const double velocity_slope_2_3 = velocity_diff / time_diff;
  // Confirm that the slope approximates the expected value kMaximumDeceleration (less than 10% difference)
  EXPECT_LT(fabs((velocity_slope_2_3 - kMaximumAcceleration) / kMaximumAcceleration), 0.1);

  // Confirm that it is decelerating linearly. The two slopes are approximate (less than 10% difference)
  EXPECT_LT(fabs((velocity_slope_2_3 - velocity_slope_1_2) / velocity_slope_1_2), 0.1);
}

/// Confirm that deceleration is not limited during sudden stops and stops immediately
TEST_F(SafetyVelocityLimiterNodeTest, SuddenStop) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to TriangleBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("triangle_bumper_test"));

  // Set to a state where there are no obstacles within the bumper range and wait for the speed to match the input
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = 10.0;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);

  // Place an obstacle in the stop area
  obstacle_point_on_base[0].x = 0.2;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);

  // Input and output speed once
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));

  // Confirm that it becomes 0 output immediately without deceleration limit
  EXPECT_DOUBLE_EQ(0.0, test_node_->output_velocity().linear.x);
}

/// Confirm that the speed limiting factor Pose published by TriangleBumper is correct
TEST_F(SafetyVelocityLimiterNodeTest, ObservedObstaclePoseTriangleBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to TriangleBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("triangle_bumper_test"));

  // Place two obstacles outside the bumper range. Do not place obstacles within the bumper range and wait for the speed to match the input
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(2, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = 10.0;
  obstacle_point_on_base[0].y = 0.0;
  obstacle_point_on_base[1].x = 0.0;
  obstacle_point_on_base[1].y = 10.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);
  // Confirm that the limiting factor Pose is not published
  ASSERT_FALSE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Place an obstacle within the bumper range
  obstacle_point_on_base[0].x = 1.0;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);

  // Input and output speed once
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is the same as the obstacle within the bumper range
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.x, obstacle_point_on_base[0].x, kEpsilon);
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.y, obstacle_point_on_base[0].y, kEpsilon);

  // Place an obstacle closer within the bumper range
  obstacle_point_on_base[1].x = 0.2;
  obstacle_point_on_base[1].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);

  // Input and output speed once
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is the same as the closer obstacle
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.x, obstacle_point_on_base[1].x, kEpsilon);
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.y, obstacle_point_on_base[1].y, kEpsilon);
}

/// Confirm that the speed limiting factor Pose published by CupBumper is correct
TEST_F(SafetyVelocityLimiterNodeTest, ObservedObstaclePoseCupBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to CupBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("cup_bumper_test"));

  // Place two obstacles outside the bumper range. Do not place obstacles within the bumper range and wait for the speed to match the input
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(2, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = 10.0;
  obstacle_point_on_base[0].y = 0.0;
  obstacle_point_on_base[1].x = 0.0;
  obstacle_point_on_base[1].y = 10.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);
  // Confirm that the limiting factor Pose is not published
  ASSERT_FALSE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Place an obstacle within the bumper range
  obstacle_point_on_base[0].x = 1.0;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);

  // Input and output speed once
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is the same as the obstacle within the bumper range
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.x, obstacle_point_on_base[0].x, kEpsilon);
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.y, obstacle_point_on_base[0].y, kEpsilon);

  // Place an obstacle closer within the bumper range
  obstacle_point_on_base[1].x = 0.2;
  obstacle_point_on_base[1].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);

  // Input and output speed once
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is the same as the closer obstacle
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.x, obstacle_point_on_base[1].x, kEpsilon);
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.y, obstacle_point_on_base[1].y, kEpsilon);
}

/// Confirm that the speed limiting factor Pose published by EllipseBumper is correct
TEST_F(SafetyVelocityLimiterNodeTest, ObservedObstaclePoseEllipseBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to EllipseBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("ellipse_bumper_test"));

  // Place two obstacles outside the bumper range. Wait for the speed to match the input
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(2, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = 10.0;
  obstacle_point_on_base[0].y = 0.0;
  obstacle_point_on_base[1].x = 0.0;
  obstacle_point_on_base[1].y = 10.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);
  // Confirm that the limiting factor Pose is not published
  ASSERT_FALSE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Move the first obstacle within the bumper range
  obstacle_point_on_base[0].x = 1.5;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);

  // Input and output speed once
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is the same as the obstacle within the bumper range
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.x, obstacle_point_on_base[0].x, kEpsilon);
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.y, obstacle_point_on_base[0].y, kEpsilon);

  // Move the second obstacle closer than the first
  obstacle_point_on_base[1].x = 1.0;
  obstacle_point_on_base[1].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);

  // Input and output speed once
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is the same as the closer obstacle
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.x, obstacle_point_on_base[1].x, kEpsilon);
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.y, obstacle_point_on_base[1].y, kEpsilon);
}

/// Confirm that the speed limiting factor Pose published by OccupancyPoint bumper is correct
TEST_F(SafetyVelocityLimiterNodeTest, ObservedObstaclePoseOccupancyPointBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to OccupancyPointBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("occupancy_point_bumper_test"));

  nav_msgs::msg::OccupancyGrid occupancy_grid;
  // Publish an OccupancyGrid with Occupancy0 around the vehicle's position
  CreateOccupancyGrid(geometry_msgs::msg::Point(), 10, 0, occupancy_grid);
  test_node_->PublishOccupancyGrid(occupancy_grid);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);
  // Confirm that the limiting factor Pose is not published
  ASSERT_FALSE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Set and publish Occupancy100 at the vehicle's position
  SetOccupancy(occupancy_grid, geometry_msgs::msg::Point(), 100);
  test_node_->PublishOccupancyGrid(occupancy_grid);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is the vehicle's position
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.x, 0.0, kEpsilon);
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.y, 0.0, kEpsilon);
}

/// Confirm that the speed limiting factor Pose published by OccupancyEllipse bumper is correct
TEST_F(SafetyVelocityLimiterNodeTest, ObservedObstaclePoseOccupancyEllipseBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to OccupancyEllipseBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("occupancy_ellipse_bumper_test"));

  // Set Occupancy value 100 outside the bumper range
  nav_msgs::msg::OccupancyGrid out_range_occupancy;
  CreateOccupancyGrid(geometry_msgs::msg::Point(), 25, 0, out_range_occupancy);
  geometry_msgs::msg::Point occupancy_point;
  occupancy_point.x = 1.0;
  occupancy_point.y = 1.0;
  SetOccupancy(out_range_occupancy, occupancy_point, 100);
  test_node_->PublishOccupancyGrid(out_range_occupancy);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  EXPECT_DOUBLE_EQ(kTestParamVelocity, test_node_->output_velocity().linear.x);
  // Confirm that the limiting factor Pose is not published
  ASSERT_FALSE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Set Occupancy value 100 within the bumper range
  nav_msgs::msg::OccupancyGrid in_range_occupancy;
  CreateOccupancyGrid(geometry_msgs::msg::Point(), 25, 0, in_range_occupancy);
  occupancy_point.x = 0.5;
  occupancy_point.y = 0.5;
  SetOccupancy(in_range_occupancy, occupancy_point, 100);
  test_node_->PublishOccupancyGrid(in_range_occupancy);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is the obstacle within the bumper range
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.x, occupancy_point.x, kEpsilon);
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.y, occupancy_point.y, kEpsilon);

  // Add Occupancy80 at the closest location within the bumper range, and Occupancy100 at the next closest location
  geometry_msgs::msg::Point occupancy_point_close;
  occupancy_point_close.x = 0.1;
  occupancy_point_close.y = 0.1;
  SetOccupancy(in_range_occupancy, occupancy_point_close, 80);
  geometry_msgs::msg::Point occupancy_point_near;
  occupancy_point_near.x = 0.2;
  occupancy_point_near.y = 0.2;
  SetOccupancy(in_range_occupancy, occupancy_point_near, 100);
  test_node_->PublishOccupancyGrid(in_range_occupancy);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is the largest Occupancy value within the bumper range and the closest obstacle
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.x, occupancy_point_near.x, kEpsilon);
  EXPECT_NEAR(test_node_->observed_obstacle_pose().pose.position.y, occupancy_point_near.y, kEpsilon);
}

/// Confirm that the size of TriangleBumper changes according to the input velocity
TEST_F(SafetyVelocityLimiterNodeTest, AutoScalingTriangleBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to AutoScaling setting TriangleBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("triangle_bumper_auto_scaling_test"));

  // Confirm that the size does not change when the input velocity is greater than or equal to max_scale_velocity
  // Place an obstacle at the edge of the bumper range
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = 1.45;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Reduce the input velocity below max_scale_velocity
  // Due to the bumper being resized, the obstacle that was within the range is now outside the range
  input_velocity_.linear.x = 0.3;
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is not published
  ASSERT_FALSE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Place an obstacle within the resized range
  obstacle_point_on_base[0].x = 0.9;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Further reduce the input velocity slightly
  // Due to the bumper being resized, the obstacle that was within the range is now outside the range
  input_velocity_.linear.x = 0.25;
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is not published
  ASSERT_FALSE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));
}

/// Confirm that the size of CupleBumper changes according to the input velocity
TEST_F(SafetyVelocityLimiterNodeTest, AutoScalingCupBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to AutoScaling setting CupBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("cup_bumper_auto_scaling_test"));

  // Confirm that the size does not change when the input velocity is greater than or equal to max_scale_velocity
  // Place an obstacle at the edge of the bumper range
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = 1.45;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Reduce the input velocity below max_scale_velocity
  // Due to the bumper being resized, the obstacle that was within the range is now outside the range
  input_velocity_.linear.x = 0.3;
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is not published
  ASSERT_FALSE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Place an obstacle within the resized range
  obstacle_point_on_base[0].x = 0.9;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Further reduce the input velocity slightly. Due to the bumper being resized, the obstacle that was within the range is now outside the range
  input_velocity_.linear.x = 0.25;
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is not published
  ASSERT_FALSE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));
}

/// Confirm that the size of EllipseBumper changes according to the input velocity
TEST_F(SafetyVelocityLimiterNodeTest, AutoScalingEllipseBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to AutoScaling setting EllipseBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("ellipse_bumper_auto_scaling_test"));

  // Confirm that the size does not change when the input velocity is greater than or equal to max_scale_velocity
  // Place an obstacle at the edge of the bumper range
  std::vector<geometry_msgs::msg::Point> obstacle_point_on_base(1, geometry_msgs::msg::Point());
  obstacle_point_on_base[0].x = 1.7;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Reduce the input velocity below max_scale_velocity
  // Due to the bumper being resized, the obstacle that was within the range is now outside the range
  input_velocity_.linear.x = 0.3;
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is not published
  ASSERT_FALSE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Place an obstacle within the resized range
  obstacle_point_on_base[0].x = 1.0;
  obstacle_point_on_base[0].y = 0.0;
  test_node_->PublishPointCloud(obstacle_point_on_base);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Further reduce the input velocity slightly. Due to the bumper being resized, the obstacle that was within the range is now outside the range
  input_velocity_.linear.x = 0.25;
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is not published
  ASSERT_FALSE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));
}

/// Confirm that the size of OccupancyEllipseBumper changes according to the input velocity
TEST_F(SafetyVelocityLimiterNodeTest, AutoScalingOccupancyEllipseBumper) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to AutoScaling setting OccupancyEllipseBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("occupancy_ellipse_bumper_auto_scaling_test"));

  // Confirm that the size does not change when the input velocity is greater than or equal to max_scale_velocity
  // Place an obstacle at the edge of the bumper range
  nav_msgs::msg::OccupancyGrid out_range_occupancy;
  CreateOccupancyGrid(geometry_msgs::msg::Point(), 25, 0, out_range_occupancy);
  geometry_msgs::msg::Point occupancy_point;
  occupancy_point.x = 0.6;
  occupancy_point.y = 0.0;
  SetOccupancy(out_range_occupancy, occupancy_point, 100);
  test_node_->PublishOccupancyGrid(out_range_occupancy);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Reduce the input velocity below max_scale_velocity
  // Due to the bumper being resized, the obstacle that was within the range is now outside the range
  input_velocity_.linear.x = 0.3;
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is not published
  ASSERT_FALSE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Place an obstacle within the resized range
  CreateOccupancyGrid(geometry_msgs::msg::Point(), 25, 0, out_range_occupancy);
  occupancy_point.x = 0.35;
  occupancy_point.y = 0.0;
  SetOccupancy(out_range_occupancy, occupancy_point, 100);
  test_node_->PublishOccupancyGrid(out_range_occupancy);
  ASSERT_TRUE(test_node_->WaitStabilization(kStabilizationTimeOut, input_velocity_));
  // Confirm that the limiting factor Pose is published
  ASSERT_TRUE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));

  // Further reduce the input velocity slightly. Due to the bumper being resized, the obstacle that was within the range is now outside the range
  input_velocity_.linear.x = 0.25;
  test_node_->PublishVelocity(input_velocity_);
  ASSERT_TRUE(test_node_->WaitForSubscribeVelocity(kSubscribeTimeOut));
  // Confirm that the limiting factor Pose is not published
  ASSERT_FALSE(test_node_->WaitForSubscribeObservedObstaclePose(kSubscribeTimeOut));
}

/// Test of the service to get current settings
TEST_F(SafetyVelocityLimiterNodeTest, GetCurrentSettingServiceTest) {
  // Enable speed limiting and switch to CupBumper
  test_node_->StartLimitVelocity();
  ASSERT_TRUE(test_node_->SwitchBumperSet("cup_bumper_test"));

  // Get current settings. Confirm that it is as set
  tmc_navigation_msgs::srv::GetCurrentSetting::Response current_setting;
  current_setting = test_node_->GetCurrentSetting();
  EXPECT_TRUE(current_setting.enable_function);
  EXPECT_EQ("cup_bumper_test", current_setting.bumper_set.data);
  EXPECT_EQ(0, current_setting.disable_bumpers.size());

  // Disable speed limiting and switch to EllipseBumper
  test_node_->StopLimitVelocity();
  ASSERT_TRUE(test_node_->SwitchBumperSet("ellipse_bumper_test"));
  // Get current settings. Confirm that the changes are reflected
  current_setting = test_node_->GetCurrentSetting();
  EXPECT_FALSE(current_setting.enable_function);
  EXPECT_EQ("ellipse_bumper_test", current_setting.bumper_set.data);
  EXPECT_EQ(0, current_setting.disable_bumpers.size());

  // Switch to a non-existent bumper
  ASSERT_FALSE(test_node_->SwitchBumperSet("undefined_bumper"));
  // Get current settings. Confirm that the set bumper has not changed
  current_setting = test_node_->GetCurrentSetting();
  EXPECT_EQ("ellipse_bumper_test", current_setting.bumper_set.data);
  EXPECT_EQ(0, current_setting.disable_bumpers.size());
}


/// Test of the service to reset to default settings
TEST_F(SafetyVelocityLimiterNodeTest, ResetToDefaultServiceTest) {
  // Enable speed limiting
  test_node_->StartLimitVelocity();
  // Switch to CupBumper
  ASSERT_TRUE(test_node_->SwitchBumperSet("cup_bumper_test"));
  // Get current settings. Confirm that it is as set
  tmc_navigation_msgs::srv::GetCurrentSetting::Response current_setting;
  current_setting = test_node_->GetCurrentSetting();
  EXPECT_TRUE(current_setting.enable_function);
  EXPECT_EQ("cup_bumper_test", current_setting.bumper_set.data);

  // Call the service to reset to default settings
  test_node_->ResetToDefault();

  // Get current settings
  // Confirm that it is as per the default settings set in test_parameters.yaml for the node under test
  const bool default_enable_function = false;
  const std::string default_bumper_set = "triangle_bumper_test";
  current_setting = test_node_->GetCurrentSetting();
  EXPECT_EQ(default_enable_function, current_setting.enable_function);
  EXPECT_EQ(default_bumper_set, current_setting.bumper_set.data);
  EXPECT_EQ(0, current_setting.disable_bumpers.size());
}

/// Test of the service to disable some bumpers
TEST_F(SafetyVelocityLimiterNodeTest, DisableBumperSettingServiceTest) {
  // Enable speed limiting and switch to a composite Bumper with CircleBumper disabled
  test_node_->StartLimitVelocity();
  std_msgs::msg::String disable_bumper;
  disable_bumper.data = "circle_bumper";
  std::vector<std_msgs::msg::String> disable_bumpers;
  disable_bumpers.push_back(disable_bumper);
  ASSERT_TRUE(test_node_->SwitchBumperSet("combination_bumper_test", disable_bumpers));

  // Get current settings. Confirm that it is as set
  tmc_navigation_msgs::srv::GetCurrentSetting::Response current_setting;
  current_setting = test_node_->GetCurrentSetting();
  EXPECT_TRUE(current_setting.enable_function);
  EXPECT_EQ("combination_bumper_test", current_setting.bumper_set.data);
  EXPECT_EQ(1, current_setting.disable_bumpers.size());
  EXPECT_EQ("circle_bumper", current_setting.disable_bumpers[0].data);

  // Switch to a non-existent bumper
  ASSERT_FALSE(test_node_->SwitchBumperSet("undefined_bumper"));

  // Get current settings. Confirm that the set bumper has not changed
  current_setting = test_node_->GetCurrentSetting();
  EXPECT_EQ("combination_bumper_test", current_setting.bumper_set.data);
  EXPECT_EQ(1, current_setting.disable_bumpers.size());
  EXPECT_EQ("circle_bumper", current_setting.disable_bumpers[0].data);

  // Call the service to reset to default settings
  test_node_->ResetToDefault();

  // Get current settings
  // Confirm that it is as per the default settings set in test_parameters.yaml for the node under test
  const bool default_enable_function = false;
  const std::string default_bumper_set = "triangle_bumper_test";
  current_setting = test_node_->GetCurrentSetting();
  EXPECT_EQ(default_enable_function, current_setting.enable_function);
  EXPECT_EQ(default_bumper_set, current_setting.bumper_set.data);
  EXPECT_EQ(0, current_setting.disable_bumpers.size());
}
}  // namespace tmc_safety_velocity_limiter

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);
  const std::string yaml_directory = ament_index_cpp::get_package_share_directory("tmc_safety_velocity_limiter") +
      "/test/parameter/";
  // Generate safety_velocity_limiter node
  auto safety_velocity_limiter_node = std::make_shared<tmc_safety_velocity_limiter::VelocityLimiter>(option);
  // Read parameters from yaml
  LoadParameterFromYaml(safety_velocity_limiter_node, yaml_directory, "test_parameters.yaml");

  safety_velocity_limiter_node->Init();
  // Start a thread to spin
  auto safety_velocity_limiter_node_thread = std::make_shared<std::thread>([&]() {
      rclcpp::spin(safety_velocity_limiter_node);
      });
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  safety_velocity_limiter_node_thread->join();
  safety_velocity_limiter_node.reset();

  return result;
}
