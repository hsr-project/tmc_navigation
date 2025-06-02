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
/// @file odometry_switcher-test.cpp
/// @brief Test for odometry switching node
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <angles/angles.h>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tmc_navigation_msgs/srv/odometry_switch.h>

#include "tmc_odometry_switcher/odometry_switcher.hpp"
#include "subscribe_rate_checker.hpp"

namespace {
/// Odometry update cycle [Hz]
const double kUpdateOdometryDefaultRate = 100.0;
const double kUpdateOdometryChangeRate = 50.0;
/// Subscriber link waiting cycle [Hz]
const double kWaitSubscriberRate = 100.0;
/// Subscriber link waiting timeout [s]
const double kWaitSubscriberTimeout = 10.0;
/// Odometry frame name
const char* const kOdomFrameName = "odom";
/// Cart frame name
const char* const kBaseFrameName = "base_footprint";
/// Odometry issue time [s]
const double kOdometryPublishPeriod = 5.0;
/// TF waiting time timeout [s]
const double kWaitTFDuration = 1.0;
/// Odometry allowable error ([m] or [rad])
const double kOdometryDiffThreshold = 0.1;
/// Odometry turning speed [rad/s]
const double kAngularVelocity = M_PI / (2.0 * kOdometryPublishPeriod);
/// Radius of the circle arc drawn by odometry [m]
const double kRadius = 1.0;

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
}  // anonymous namespace

namespace tmc_odometry_switcher {
using std::placeholders::_1;

/// Odometry publisher class for testing
/// Specify topic name and rotation direction in constructor arguments
class TestOdometryPublisher {
 public:
  typedef std::shared_ptr<TestOdometryPublisher> Ptr;
  TestOdometryPublisher(rclcpp::Node::SharedPtr node,
                    const std::string& topic_name,
                    const bool move_clockwize)
      : odometry_(nav_msgs::msg::Odometry()),
        move_clockwize_(move_clockwize) {
    // Publisher initialization
    odometry_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>(topic_name, 1);

    // Wait until link with subscriber is established
    rclcpp::Rate wait_rate(kWaitSubscriberRate);
    rclcpp::Time wait_start = rclcpp::Clock(RCL_ROS_TIME).now();
    while (odometry_publisher_->get_subscription_count() == 0) {
      wait_rate.sleep();
      if (rclcpp::Clock(RCL_ROS_TIME).now() - wait_start > rclcpp::Duration::from_seconds(kWaitSubscriberTimeout)) {
        RCLCPP_ERROR(node->get_logger(), "Odometry subscriber is not exist.");
        exit(EXIT_FAILURE);
      }
    }
    odometry_.header.frame_id = kOdomFrameName;
    odometry_.pose.pose.orientation.w = 1.0;
  }
  ~TestOdometryPublisher() {}

  // Update and publish odometry for specified time
  // Perform circular motion with radius kRadius[m] starting from origin
  // Switch between clockwise/anti-clockwise using move_clockwize_
  void UpdateOdometry(const double period) {
    // Switch clockwise/anti-clockwise
    double direction = move_clockwize_ ? -1.0 : 1.0;
    // Odometry update
    double yaw = tf2::getYaw(odometry_.pose.pose.orientation);
    odometry_.pose.pose.position.x += kAngularVelocity * kRadius * cos(yaw) * period;
    odometry_.pose.pose.position.y += kAngularVelocity * kRadius * sin(yaw) * period;
    yaw += direction * kAngularVelocity * period;
    odometry_.pose.pose.orientation = createQuaternionMsgFromYaw(yaw);
    odometry_.twist.twist.linear.x = kAngularVelocity * kRadius;
    odometry_.twist.twist.angular.z = direction * kAngularVelocity;
    // Publish odometry
    odometry_.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    odometry_publisher_->publish(odometry_);
  }

 private:
  // Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  // Current odometry
  nav_msgs::msg::Odometry odometry_;
  // Odometry movement direction (clockwise or anti-clockwise)
  bool move_clockwize_;
};


/// Test node class
class TestNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {}

  /// Initialization
  void Init() {
    SetRate(kUpdateOdometryDefaultRate);
    // Generate odometry topic publisher 1 (odometry moving in circle anti-clockwise)
    TestOdometryPublisher::Ptr publisher1(new TestOdometryPublisher(shared_from_this(), "odom1", false));
    odometry_publishers_.push_back(publisher1);
    // Generate odometry topic publisher 2 (odometry moving in circle clockwise)
    TestOdometryPublisher::Ptr publisher2(new TestOdometryPublisher(shared_from_this(), "odom2", true));
    odometry_publishers_.push_back(publisher2);
    // Generate odometry subscriber
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "switched_odom", 1,
        std::bind(&TestNode::OdometryCallback, this, _1));
    // Subscriber for odometry reception cycle
    rate_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        "switched_odom_subscribe_rate", 1,
        std::bind(&TestNode::SubscribeRateCallback, this, _1));

    // Service client initial setup
    odometry_switch_client_ = this->create_client<tmc_navigation_msgs::srv::OdometrySwitch>("odometry_switch");
    start_check_rate_client_ = this->create_client<std_srvs::srv::Empty>("start_check_rate");
    stop_check_rate_client_ = this->create_client<std_srvs::srv::Empty>("stop_check_rate");
  }

  /// Spin the test node
  void SpinOnce() {
    rclcpp::spin_some(shared_from_this());
  }

  /// Switch odometry sources
  bool SwitchOdom(const std::string& odom_type) {
    auto request = std::make_shared<tmc_navigation_msgs::srv::OdometrySwitch::Request>();
    request->odom_type.data = odom_type;
    auto result_future = odometry_switch_client_->async_send_request(request);
    rclcpp::spin_until_future_complete(shared_from_this(), result_future);
    return result_future.get()->is_success;
  }

  /// Start cycle check
  void StartCheckRate() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result_future = start_check_rate_client_->async_send_request(request);
    rclcpp::spin_until_future_complete(shared_from_this(), result_future);
  }

  /// Stop cycle check
  void StopCheckRate() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result_future = stop_check_rate_client_->async_send_request(request);
    rclcpp::spin_until_future_complete(shared_from_this(), result_future);
  }

  /// Publish odometry for specified time
  void PublishOdometries(const double period) {
    rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
    while ((rclcpp::Clock(RCL_ROS_TIME).now() - start_time).seconds() <= period) {
      for (std::vector<TestOdometryPublisher::Ptr>::iterator it = odometry_publishers_.begin();
           it != odometry_publishers_.end(); ++it) {
        const double dt = static_cast<double>(rate_->period().count()) * 1e-9;
        (*it)->UpdateOdometry(dt);
      }
      SpinOnce();
      rate_->sleep();
    }
  }

  /// Run cycle without doing anything for specified time
  void RunWithoutDoAnything(const double period) {
    rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
    while ((rclcpp::Clock(RCL_ROS_TIME).now() - start_time).seconds() <= period) {
      SpinOnce();
      rate_->sleep();
    }
  }

  /// Get Transform
  geometry_msgs::msg::TransformStamped LookupTransform(
      const std::string& target_frame, const std::string& source_frame,
      const rclcpp::Time& stamp, const rclcpp::Duration& timeout) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer_.lookupTransform(target_frame, source_frame, stamp, timeout);
    return transform_stamped;
  }

  /// Change cycle
  void SetRate(const double rate) {
    rate_ = std::make_shared<rclcpp::Rate>(rate);
  }

  void ClearSubscribeRate() { subscribe_rate_ = 0.0;}

  nav_msgs::msg::Odometry odometry() const { return odometry_; }
  double subscribe_rate() const { return subscribe_rate_; }

 private:
  // Odometry callback
  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odometry_ = *msg;
  }

  // Odometry reception cycle callback
  void SubscribeRateCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    subscribe_rate_ = msg->data;
  }

  // Obtain odometry
  nav_msgs::msg::Odometry odometry_;
  // Odometry update cycle
  std::shared_ptr<rclcpp::Rate> rate_;
  // tf buffer
  tf2_ros::Buffer tf_buffer_;
  // tf listener
  tf2_ros::TransformListener tf_listener_;
  // Odometry publisher
  std::vector<TestOdometryPublisher::Ptr> odometry_publishers_;
  // Odometry subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  // Subscriber for odometry reception cycle
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rate_subscriber_;
  // Odometry switching service client
  rclcpp::Client<tmc_navigation_msgs::srv::OdometrySwitch>::SharedPtr odometry_switch_client_;
  // Service client to start odometry reception cycle check
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_check_rate_client_;
  // Service client to stop odometry reception cycle check
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_check_rate_client_;
  // Odometry reception cycle
  double subscribe_rate_;
};

/// Test class (odometry_switcher_test)
class OdometrySwitcherTest : public testing::Test {
 public:
  OdometrySwitcherTest() {}
  virtual ~OdometrySwitcherTest() = default;

 protected:
  virtual void SetUp() {
    // Generate test node
    test_node_ = std::make_shared<TestNode>();
    test_node_->Init();
    // Reset switcher's input odometry to odometry 1
    ASSERT_TRUE(test_node_->SwitchOdom("test1_odom"));
  }
  virtual void TearDown() {}

  std::shared_ptr<TestNode> test_node_;
};

/// Switching test of 2D odometry
/// Check if the service to switch input odometry succeeds
/// Check if the result is reflected in the output odometry
/// Check if the output odometry is calculated as a relative value during switching, not the absolute value of the input odometry
TEST_F(OdometrySwitcherTest, Switch2DOdometry) {
  // Publish odometry for specified time with odometry 1 input setup
  // Rotate one-fourth of a circle anti-clockwise
  test_node_->PublishOdometries(kOdometryPublishPeriod);
  nav_msgs::msg::Odometry odometry = test_node_->odometry();

  // Compare expected odometry topic with actual odometry topic
  // After rotating one-fourth of a circle anti-clockwise, x and y should both take the radius value and yaw should be PI/2
  double expected_x = kRadius;
  double expected_y = kRadius;
  double expected_yaw = M_PI / 2.0;
  double current_odom_x = odometry.pose.pose.position.x;
  double current_odom_y = odometry.pose.pose.position.y;
  double current_odom_yaw = tf2::getYaw(odometry.pose.pose.orientation);
  EXPECT_LT(fabs(current_odom_x - expected_x), kOdometryDiffThreshold);
  EXPECT_LT(fabs(current_odom_y - expected_y), kOdometryDiffThreshold);
  EXPECT_LT(angles::shortest_angular_distance(current_odom_yaw, expected_yaw), kOdometryDiffThreshold);

  // Compare expected odometry TF with actual odometry TF
  geometry_msgs::msg::TransformStamped transform_stamped;
  ASSERT_NO_THROW({
      transform_stamped = test_node_->LookupTransform(kOdomFrameName,
                                                      kBaseFrameName,
                                                      odometry.header.stamp,
                                                      rclcpp::Duration::from_seconds(kWaitTFDuration));
    });
  double current_tf_x = transform_stamped.transform.translation.x;
  double current_tf_y = transform_stamped.transform.translation.y;
  double current_tf_yaw = tf2::getYaw(transform_stamped.transform.rotation);
  EXPECT_LT(fabs(current_tf_x - expected_x), kOdometryDiffThreshold);
  EXPECT_LT(fabs(current_tf_y - expected_y), kOdometryDiffThreshold);
  EXPECT_LT(angles::shortest_angular_distance(current_tf_yaw, expected_yaw), kOdometryDiffThreshold);

  // Check if it can switch to odometry 2
  ASSERT_TRUE(test_node_->SwitchOdom("test2_odom"));

  // Publish odometry for specified time with odometry 2 input setup
  // Rotate one-fourth of a circle clockwise
  test_node_->PublishOdometries(kOdometryPublishPeriod);
  odometry = test_node_->odometry();
  // Compare expected odometry topic with actual odometry topic
  // The output odometry should be the relative change added since the odometry switch
  // After moving one-fourth of a circle anti-clockwise, then connecting the trajectory of one-fourth of a circle clockwise
  // It should take an S-shaped trajectory, with x, y taking twice the radius value, and yaw should become zero
  expected_x += kRadius;
  expected_y += kRadius;
  expected_yaw -= M_PI/ 2.0;
  current_odom_x = odometry.pose.pose.position.x;
  current_odom_y = odometry.pose.pose.position.y;
  current_odom_yaw = tf2::getYaw(odometry.pose.pose.orientation);
  EXPECT_LT(fabs(current_odom_x - expected_x), kOdometryDiffThreshold);
  EXPECT_LT(fabs(current_odom_y - expected_y), kOdometryDiffThreshold);
  EXPECT_LT(angles::shortest_angular_distance(current_odom_yaw, expected_yaw), kOdometryDiffThreshold);

  // Compare expected odometry TF with actual odometry TF
  ASSERT_NO_THROW({
      transform_stamped = test_node_->LookupTransform(kOdomFrameName,
                                                      kBaseFrameName,
                                                      odometry.header.stamp,
                                                      rclcpp::Duration::from_seconds(kWaitTFDuration));
    });
  current_tf_x = transform_stamped.transform.translation.x;
  current_tf_y = transform_stamped.transform.translation.y;
  current_tf_yaw = tf2::getYaw(transform_stamped.transform.rotation);
  EXPECT_LT(fabs(current_tf_x - expected_x), kOdometryDiffThreshold);
  EXPECT_LT(fabs(current_tf_y - expected_y), kOdometryDiffThreshold);
  EXPECT_LT(angles::shortest_angular_distance(current_tf_yaw, expected_yaw), kOdometryDiffThreshold);
}


/// Output cycle test
/// Confirm it becomes a cycle according to input
TEST_F(OdometrySwitcherTest, OutputRate) {
  // Cycle is kUpdateOdometryDefaultRate
  test_node_->SetRate(kUpdateOdometryDefaultRate);
  test_node_->StartCheckRate();
  // Publish odometry for specified time
  test_node_->PublishOdometries(kOdometryPublishPeriod);
  // Run cycle to process remaining topics in the queue
  test_node_->RunWithoutDoAnything(3.0);
  // Published cycle and subscribed cycle should be close (less than 5% difference)
  EXPECT_LT(fabs(kUpdateOdometryDefaultRate - test_node_->subscribe_rate()) / kUpdateOdometryDefaultRate, 0.05);
  test_node_->StopCheckRate();

  // Change cycle to kUpdateOdometryChangeRate
  test_node_->SetRate(kUpdateOdometryChangeRate);
  test_node_->StartCheckRate();
  test_node_->ClearSubscribeRate();
  // Publish odometry for specified time
  test_node_->PublishOdometries(kOdometryPublishPeriod);
  // Run cycle to process remaining topics in the queue
  test_node_->RunWithoutDoAnything(3.0);
  // Published cycle and subscribed cycle should be close (less than 5% difference)
  EXPECT_LT(fabs(kUpdateOdometryChangeRate - test_node_->subscribe_rate()) / kUpdateOdometryChangeRate, 0.05);
  test_node_->StopCheckRate();

  // When there is no publish
  test_node_->StartCheckRate();
  test_node_->ClearSubscribeRate();
  test_node_->RunWithoutDoAnything(kOdometryPublishPeriod);
  // There should be no callback while there is no publish
  EXPECT_LT(test_node_->subscribe_rate(), std::numeric_limits<double>::epsilon());
  test_node_->StopCheckRate();
}

/// Odometry switching test
/// Check if specifying a name not in the list returns a failure
TEST_F(OdometrySwitcherTest, WrongOdomName) {
  EXPECT_FALSE(test_node_->SwitchOdom("wrong_odom"));
}
}  // namespace tmc_odometry_switcher


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);
  const std::string yaml_directory = ament_index_cpp::get_package_share_directory("tmc_odometry_switcher") +
      "/test/parameter/";
  // Generate odometry_switcher node
  auto odometry_switcher_node = std::make_shared<tmc_odometry_switcher::OdometrySwitcherNode>(option);
  // Read parameters from yaml
  LoadParameterFromYaml(odometry_switcher_node, yaml_directory, "odometry_switcher-test.yaml");

  odometry_switcher_node->Init();
  // Create a thread to spin and detach
  auto odometry_switcher_node_thread = std::make_shared<std::thread>([&]() {
      rclcpp::spin(odometry_switcher_node);
      });
  odometry_switcher_node_thread->detach();
  // Generate subscribe_rate_checker node
  auto subscribe_rate_checker_node = std::make_shared<tmc_odometry_switcher::SubscribeRateChecker>(option);
  subscribe_rate_checker_node->Init();
  // Create a thread to spin and detach
  auto subscribe_rate_checker_node_thread = std::make_shared<std::thread>([&]() {
      rclcpp::spin(subscribe_rate_checker_node);
      });
  subscribe_rate_checker_node_thread->detach();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
