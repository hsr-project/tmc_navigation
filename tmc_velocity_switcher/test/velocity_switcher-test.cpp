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
/// Node test for velocity_switcher
/// Check Publish, Subscribe, and input/output.
/// Copyright (C) 2023 TOYOTA Motor Corporation.
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/parameter_map.hpp>
#include "velocity_switcher-test.hpp"

namespace {

// Comparison threshold
constexpr double kCompareThreshold = 0.0001;

// Test update cycle [hz]
constexpr double kTestRate = 400.0;

// Timeout duration [sec]
constexpr double kTimeOutSec = 5.0;

// Topic buffer size
constexpr int32_t kTopicBufferSize = 1000;

// Output velocity topic name
constexpr const char* const kNameOutputVelocity = "output_velocity";

/// Variables used in the target node for testing Ensure consistency with the target node
// Operation cycle of the target node [hz]
constexpr double kControlCycleTestNode    = 200.0;
// Velocity switching time of the target node [sec]
constexpr double kPeriodSwitchingTestNode = 0.50;

// Test pattern configuration velocity [m/s]
constexpr double kTestParamVelocity0 = 1.0;
constexpr double kTestParamVelocity1 = 2.0;
constexpr double kTestParamVelocity2 = 3.0;
constexpr double kTestParamVelocity3 = 4.0;
constexpr double kTestParamVelocity4 = 5.0;
constexpr double kTestParamVelocity5 = 6.0;
constexpr double kTestParamVelocity6 = 7.0;

// Loop count to consider output_velocity value as stable
constexpr int32_t kTestParamLoopCountThresholdToCheckConstant = 100;

// Input velocity topic name
constexpr const char* const kNameInputVelocity0 = "input_velocity0";
constexpr const char* const kNameInputVelocity1 = "input_velocity1";
constexpr const char* const kNameInputVelocity2 = "input_velocity2";
constexpr const char* const kNameInputVelocity3 = "input_velocity3";
constexpr const char* const kNameInputVelocity4 = "input_velocity4";
constexpr const char* const kNameInputVelocity5 = "input_velocity5";
constexpr const char* const kNameInputVelocity6 = "input_velocity6";

// Enum to manage test pattern numbers
enum kTestPatternEnum {
  kTestPatternEnum0 = 0,
  kTestPatternEnum1 = 1,
  kTestPatternEnum2 = 2,
  kTestPatternEnum3 = 3,
  kTestPatternEnum4 = 4,
  kTestPatternEnum5 = 5,
  kTestPatternEnum6 = 6,
  kTestPatternEnum7 = 7,
  kTestPatternEnum8 = 8,
  kTestPatternEnum9 = 9,
  kTestPatternEnum10 = 10,
};

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
  // Set ROS parameters to the node
  const std::string parameter_space = "/" + std::string(node->get_name());
  auto iter = yaml_param_map.find(parameter_space);
  for (auto& param : iter->second) {
    node->declare_parameter(param.get_name(), param.get_type());
    node->set_parameter(param);
  }
}
}  // anonymous namespace


namespace tmc_velocity_switcher {
using std::placeholders::_1;
// Constructor
TestNode::TestNode(const rclcpp::NodeOptions& options):
    Node("test_node", options),
    subscribed_flag_(false) {}

// Initialization
void TestNode::Init() {
  // Reception waiting flag true = reception complete false = waiting for reception
  subscribed_flag_ = false;

  // Declaration of topic to publish
  velocity0_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(kNameInputVelocity0, kTopicBufferSize);
  velocity1_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(kNameInputVelocity1, kTopicBufferSize);
  velocity2_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(kNameInputVelocity2, kTopicBufferSize);
  velocity3_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(kNameInputVelocity3, kTopicBufferSize);
  velocity4_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(kNameInputVelocity4, kTopicBufferSize);
  velocity5_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(kNameInputVelocity5, kTopicBufferSize);
  velocity6_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(kNameInputVelocity6, kTopicBufferSize);

  // Subscribe definition
  velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        kNameOutputVelocity, kTopicBufferSize,
        std::bind(&TestNode::CallbackOutput, this, _1));

  // Publisher construction test
  ASSERT_TRUE(WaitForPublishersLinked());
  // Subscriber construction test
  ASSERT_TRUE(WaitForSubscriberLinked());
  // Test parameter settings
  SetTestParam();
}

// Timeout check Returns true if timed out
bool TestNode::CheckTimeOut(const rclcpp::Time& start_time) const {
  const double elapsed_time = (rclcpp::Clock(RCL_ROS_TIME).now() - start_time).seconds();
  return (kTimeOutSec < elapsed_time);
}

// Function to wait for publisher link
bool TestNode::WaitForPublishersLinked() {
  rclcpp::Rate rate(kTestRate);
  const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
  while (rclcpp::ok() &&
      (velocity0_pub_->get_subscription_count() == 0 ||
       velocity1_pub_->get_subscription_count() == 0 ||
       velocity2_pub_->get_subscription_count() == 0 ||
       velocity3_pub_->get_subscription_count() == 0 ||
       velocity4_pub_->get_subscription_count() == 0 ||
       velocity5_pub_->get_subscription_count() == 0 ||
       velocity6_pub_->get_subscription_count() == 0)) {
    if (CheckTimeOut(start_time)) {
      // Failure if timed out
      return false;
    }
    SpinOnce();
    rate.sleep();
  }
  return true;
}

// Function to wait for subscriber link
bool TestNode::WaitForSubscriberLinked() {
  // Wait for link with publisher
  rclcpp::Rate rate(kTestRate);
  const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
  while (velocity_sub_->get_publisher_count() <= 0) {
    if (CheckTimeOut(start_time)) {
      return false;
    }
    SpinOnce();
    rate.sleep();
  }
  return true;
}

// Function to wait for subscriber reception
bool TestNode::WaitForSubscriberReceived() {
  rclcpp::Rate rate(kTestRate);
  const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
  // Waiting for reception
  while (!subscribed_flag_ && rclcpp::ok()) {
    if (CheckTimeOut(start_time)) {
      return false;
    }
    SpinOnce();
    rate.sleep();
  }

  return true;
}


// Wait until output_velocity value stabilizes at 0.0
void TestNode::WaitUntilOutputGetsZero() {
  rclcpp::Rate rate(kTestRate);
  while (rclcpp::ok()) {
    if (fabs(output_velocity_.linear.x) < kCompareThreshold) {
      break;
    }
    SpinOnce();
    rate.sleep();
  }
}

// Callback function for output_velocity
void TestNode::CallbackOutput(const geometry_msgs::msg::Twist::SharedPtr msg) {
  output_velocity_ = *msg;
  subscribed_flag_ = true;
}

/// Monitor output_velocity's linear.x for a certain period
/// Check the first change in its value
bool TestNode::CheckSubscribedDataChange() {
  rclcpp::Rate rate(kTestRate);
  const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
  bool data_changed = false;
  double pre_data = output_velocity_.linear.x;

  // Change check
  while (!CheckTimeOut(start_time) && rclcpp::ok()) {
    // Compare previous and current values Return TRUE if values differ
    if (fabs(pre_data - output_velocity_.linear.x) > kCompareThreshold) {
      return true;
    }
    // Store current value
    pre_data = output_velocity_.linear.x;
    SpinOnce();
    rate.sleep();
  }
  return false;
}

// Setup
void VelocitySwitcherNodeTest::SetUp() {
  test_node_ = std::make_shared<TestNode>();
  test_node_->Init();
}

// Teardown
void VelocitySwitcherNodeTest::TearDown() {
  test_node_->WaitUntilOutputGetsZero();
}

// Publish input velocity 0
void TestNode::PublishVelocity0() {
  velocity0_pub_->publish(input_velocity0_);
}

// Publish input velocity 1
void TestNode::PublishVelocity1() {
  velocity1_pub_->publish(input_velocity1_);
}

// Publish input velocity 2
void TestNode::PublishVelocity2() {
  velocity2_pub_->publish(input_velocity2_);
}

// Publish input velocity 3
void TestNode::PublishVelocity3() {
  velocity3_pub_->publish(input_velocity3_);
}

// Publish input velocity 4
void TestNode::PublishVelocity4() {
  velocity4_pub_->publish(input_velocity4_);
}

// Publish input velocity 5
void TestNode::PublishVelocity5() {
  velocity5_pub_->publish(input_velocity5_);
}

// Publish input velocity 6
void TestNode::PublishVelocity6() {
  velocity6_pub_->publish(input_velocity6_);
}

// spin
void TestNode::SpinOnce() {
  rclcpp::spin_some(shared_from_this());
}

// Test parameter settings
void TestNode::SetTestParam() {
  input_velocity0_.linear.x  = kTestParamVelocity0;
  input_velocity0_.linear.y  = kTestParamVelocity0;
  input_velocity0_.linear.z  = kTestParamVelocity0;
  input_velocity0_.angular.x = kTestParamVelocity0;
  input_velocity0_.angular.y = kTestParamVelocity0;
  input_velocity0_.angular.z = kTestParamVelocity0;

  input_velocity1_.linear.x  = kTestParamVelocity1;
  input_velocity1_.linear.y  = kTestParamVelocity1;
  input_velocity1_.linear.z  = kTestParamVelocity1;
  input_velocity1_.angular.x = kTestParamVelocity1;
  input_velocity1_.angular.y = kTestParamVelocity1;
  input_velocity1_.angular.z = kTestParamVelocity1;

  input_velocity2_.linear.x  = kTestParamVelocity2;
  input_velocity2_.linear.y  = kTestParamVelocity2;
  input_velocity2_.linear.z  = kTestParamVelocity2;
  input_velocity2_.angular.x = kTestParamVelocity2;
  input_velocity2_.angular.y = kTestParamVelocity2;
  input_velocity2_.angular.z = kTestParamVelocity2;

  input_velocity3_.linear.x  = kTestParamVelocity3;
  input_velocity3_.linear.y  = kTestParamVelocity3;
  input_velocity3_.linear.z  = kTestParamVelocity3;
  input_velocity3_.angular.x = kTestParamVelocity3;
  input_velocity3_.angular.y = kTestParamVelocity3;
  input_velocity3_.angular.z = kTestParamVelocity3;

  input_velocity4_.linear.x  = kTestParamVelocity4;
  input_velocity4_.linear.y  = kTestParamVelocity4;
  input_velocity4_.linear.z  = kTestParamVelocity4;
  input_velocity4_.angular.x = kTestParamVelocity4;
  input_velocity4_.angular.y = kTestParamVelocity4;
  input_velocity4_.angular.z = kTestParamVelocity4;

  input_velocity5_.linear.x  = kTestParamVelocity5;
  input_velocity5_.linear.y  = kTestParamVelocity5;
  input_velocity5_.linear.z  = kTestParamVelocity5;
  input_velocity5_.angular.x = kTestParamVelocity5;
  input_velocity5_.angular.y = kTestParamVelocity5;
  input_velocity5_.angular.z = kTestParamVelocity5;

  input_velocity6_.linear.x  = kTestParamVelocity6;
  input_velocity6_.linear.y  = kTestParamVelocity6;
  input_velocity6_.linear.z  = kTestParamVelocity6;
  input_velocity6_.angular.x = kTestParamVelocity6;
  input_velocity6_.angular.y = kTestParamVelocity6;
  input_velocity6_.angular.z = kTestParamVelocity6;
}

// Function to publish test patterns based on test_pattern_num
void VelocitySwitcherNodeTest::PubTopic(const int32_t test_pattern_num) {
  switch (test_pattern_num) {
    case kTestPatternEnum0:
      test_node_->PublishVelocity0();
      break;
    case kTestPatternEnum1:
      test_node_->PublishVelocity1();
      break;
    case kTestPatternEnum2:
      test_node_->PublishVelocity2();
      break;
    case kTestPatternEnum3:
      test_node_->PublishVelocity3();
      break;
    case kTestPatternEnum4:
      test_node_->PublishVelocity4();
      break;
    case kTestPatternEnum5:
      test_node_->PublishVelocity0();
      test_node_->PublishVelocity1();
      break;
    case kTestPatternEnum6:
      test_node_->PublishVelocity0();
      test_node_->PublishVelocity1();
      test_node_->PublishVelocity2();
      break;
    case kTestPatternEnum7:
      test_node_->PublishVelocity0();
      test_node_->PublishVelocity1();
      test_node_->PublishVelocity2();
      test_node_->PublishVelocity3();
      break;
    case kTestPatternEnum8:
      test_node_->PublishVelocity0();
      test_node_->PublishVelocity1();
      test_node_->PublishVelocity2();
      test_node_->PublishVelocity3();
      test_node_->PublishVelocity4();
      break;
    case kTestPatternEnum9:
      test_node_->PublishVelocity0();
      test_node_->PublishVelocity5();
      break;
    case kTestPatternEnum10:
      test_node_->PublishVelocity0();
      test_node_->PublishVelocity6();
      break;
    default:
      break;
  }
  return;
}

INSTANTIATE_TEST_CASE_P(
    VelocitySwitcherNodeTestFirstValue,
    VelocitySwitcherNodeTest,
    testing::Values(std::make_pair(kTestPatternEnum0, kTestParamVelocity0),
                    std::make_pair(kTestPatternEnum1, kTestParamVelocity1),
                    std::make_pair(kTestPatternEnum2, kTestParamVelocity2),
                    std::make_pair(kTestPatternEnum3, kTestParamVelocity3),
                    std::make_pair(kTestPatternEnum4, kTestParamVelocity4),
                    std::make_pair(kTestPatternEnum5, kTestParamVelocity0),
                    std::make_pair(kTestPatternEnum6, kTestParamVelocity0),
                    std::make_pair(kTestPatternEnum7, kTestParamVelocity0),
                    std::make_pair(kTestPatternEnum8, kTestParamVelocity0)
));

/// Test
/// Obtain input_velocity and test whether the first changed value of the output_velocity
/// matches the expected value
TEST_P(VelocitySwitcherNodeTest, first_value) {
  PubTopic(GetParam().first);
  // Check if the value has changed
  ASSERT_TRUE(test_node_->CheckSubscribedDataChange());
  /// The target node operates at kControlCycleTestNode [hz]
  /// and transitions to the maximum value in kPeriodSwitchingTestNode [sec]
  /// In one step
  /// The speed increases by Maximum value/kControlCycleTestNode/kPeriodSwitchingTestNode
  /// In this test, the first subscribed value is examined
  /// That value should be
  /// Maximum value/kControlCycleTestNode/kPeriodSwitchingTestNode
  /// to pass
  // Obtain continuously output input_velocity0
  EXPECT_EQ(GetParam().second/kControlCycleTestNode / kPeriodSwitchingTestNode,
            test_node_->output_velocity().linear.x);
  EXPECT_EQ(GetParam().second/kControlCycleTestNode / kPeriodSwitchingTestNode,
            test_node_->output_velocity().linear.y);
  EXPECT_EQ(GetParam().second/kControlCycleTestNode / kPeriodSwitchingTestNode,
            test_node_->output_velocity().angular.z);
}


// Test whether the output_velocity value stabilizes at the input_velocity0 value
// Output input_velocity0
TEST_F(VelocitySwitcherNodeTest, Continuation_input_each_publisher0) {
  rclcpp::Rate rate(kTestRate);
  const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();

  bool subscribed_data_changed = false;
  double pre_data = test_node_->output_velocity().linear.x;
  int32_t loop_count = 0;

  while (rclcpp::ok()) {
    // Set flag ON when published value starts changing
    PubTopic(kTestPatternEnum0);

    // If flag is ON, the value has started moving
    if (!subscribed_data_changed &&
        fabs(pre_data - test_node_->output_velocity().linear.x) > kCompareThreshold) {
      subscribed_data_changed = true;
    }

    // Previous and current values are the same
    if (subscribed_data_changed) {
      // Values remain the same for a certain period
      if (fabs(pre_data - test_node_->output_velocity().linear.x) < kCompareThreshold) {
        // Consider the value stable and perform the test
        if (loop_count > kTestParamLoopCountThresholdToCheckConstant) {
          // Timeout check
          EXPECT_EQ(kTestParamVelocity0, test_node_->output_velocity().linear.x);
          break;
        }
        loop_count++;
      }
    }
    // Store current value
    ASSERT_FALSE(test_node_->CheckTimeOut(start_time));
    // Obtain continuously output input_velocity1
    pre_data = test_node_->output_velocity().linear.x;
    test_node_->SpinOnce();
    rate.sleep();
  }
}

// Test whether the output_velocity value stabilizes at the input_velocity1 value
// Output input_velocity1
TEST_F(VelocitySwitcherNodeTest, Continuation_input_each_publisher1) {
  rclcpp::Rate rate(kTestRate);
  const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();

  bool subscribed_data_changed = false;
  double pre_data = test_node_->output_velocity().linear.x;
  int32_t loop_count = 0;

  while (rclcpp::ok()) {
    // Set flag ON when published value starts changing
    PubTopic(kTestPatternEnum1);

    // If flag is ON, the value has started moving
    if (!subscribed_data_changed &&
        fabs(pre_data - test_node_->output_velocity().linear.x) > kCompareThreshold) {
      subscribed_data_changed = true;
    }

    // Previous and current values are the same
    if (subscribed_data_changed) {
      // Values remain the same for a certain period
      if (fabs(pre_data - test_node_->output_velocity().linear.x) < kCompareThreshold) {
        // Consider the value stable and perform the test
        if (loop_count > kTestParamLoopCountThresholdToCheckConstant) {
          // Timeout check
          EXPECT_EQ(kTestParamVelocity1, test_node_->output_velocity().linear.x);
          break;
        }
        loop_count++;
      }
    }
    // Store current value
    ASSERT_FALSE(test_node_->CheckTimeOut(start_time));
    /// Obtain continuously output input_velocity0 and 1
    pre_data = test_node_->output_velocity().linear.x;
    test_node_->SpinOnce();
    rate.sleep();
  }
}

/// Stop input_velocity0 when output_velocity stabilizes
/// Test whether the value stabilizes at input_velocity1
// Provide input_velocity0
TEST_F(VelocitySwitcherNodeTest, Continuation_input_composite_publisher) {
  rclcpp::Rate rate(kTestRate);
  rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();

  bool subscribed_data_changed = false;
  double pre_data = test_node_->output_velocity().linear.x;
  int32_t loop_count = 0;

  // Set flag ON when published value starts changing
  while (rclcpp::ok()) {
    PubTopic(kTestPatternEnum5);

    // If flag is ON, the value has started moving
    if (!subscribed_data_changed &&
        fabs(pre_data - test_node_->output_velocity().linear.x) > kCompareThreshold) {
      subscribed_data_changed = true;
    }


    // Previous and current values are the same
    if (subscribed_data_changed) {
      // Values remain the same for a certain period
      if (fabs(pre_data - test_node_->output_velocity().linear.x) < kCompareThreshold) {
        // Consider the value stable and perform the test
        if (loop_count > kTestParamLoopCountThresholdToCheckConstant) {
          } else {  // Reset if the value changes. Values must remain the same continuously
          break;
        }
        loop_count++;
      // Timeout check
        loop_count = 0;
      }
    }
    // Store current value
    ASSERT_FALSE(test_node_->CheckTimeOut(start_time));
    // Update start time
    pre_data = test_node_->output_velocity().linear.x;
    test_node_->SpinOnce();
    rate.sleep();
  }


  // Provide input_velocity1
  start_time = rclcpp::Clock(RCL_ROS_TIME).now();
  subscribed_data_changed = false;

  // Set flag ON when published value starts changing
  while (rclcpp::ok()) {
    PubTopic(kTestPatternEnum1);

    /// Gradually change from kTestParamVelocity0 to kTestParamVelocity1
    if (!subscribed_data_changed &&
        fabs(pre_data - test_node_->output_velocity().linear.x) > kCompareThreshold) {
      /// Obtain the first change and test
      // If flag is ON, the value has started moving
      EXPECT_EQ((kTestParamVelocity1 - kTestParamVelocity0) /
                kControlCycleTestNode/kPeriodSwitchingTestNode +
                kTestParamVelocity0,
                test_node_->output_velocity().linear.x);

      subscribed_data_changed = true;
    }

    // Previous and current values are the same
    if (subscribed_data_changed) {
      // Values remain the same for a certain period
      if (fabs(pre_data - test_node_->output_velocity().linear.x) < kCompareThreshold) {
        // Consider the value stable and perform the test
        if (loop_count > kTestParamLoopCountThresholdToCheckConstant) {
          } else {  // Reset if the value changes. Values must remain the same continuously
          EXPECT_EQ(kTestParamVelocity1, test_node_->output_velocity().linear.x);
          break;
        }
        loop_count++;
      // Timeout check
        loop_count = 0;
      }
    }
    // Store current value
    ASSERT_FALSE(test_node_->CheckTimeOut(start_time));
    // For input_velocity5 controlling only the XY axis with high priority
    pre_data = test_node_->output_velocity().linear.x;
    test_node_->SpinOnce();
    rate.sleep();
  }
}

// and input_velocity0 controlling all axes with low priority
// Test whether the XY axis of output_velocity stabilizes at input_velocity5
// and the rotational axis stabilizes at input_velocity0
// Set flag ON when published value starts changing
TEST_F(VelocitySwitcherNodeTest, Control_specific_axis0) {
  rclcpp::Rate rate(kTestRate);
  const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();

  bool subscribed_data_changed = false;
  double pre_data = test_node_->output_velocity().linear.x;
  int32_t loop_count = 0;

  while (rclcpp::ok()) {
    PubTopic(kTestPatternEnum9);

    // If flag is ON, the value has started moving
    if (!subscribed_data_changed &&
        fabs(pre_data - test_node_->output_velocity().linear.x) > kCompareThreshold) {
      subscribed_data_changed = true;
    }

    // Previous and current values are the same
    if (subscribed_data_changed) {
      // Values remain the same for a certain period
      if (fabs(pre_data - test_node_->output_velocity().linear.x) < kCompareThreshold) {
        // Consider the value stable and perform the test
        if (loop_count > kTestParamLoopCountThresholdToCheckConstant) {
          // Timeout check
          EXPECT_EQ(kTestParamVelocity5, test_node_->output_velocity().linear.x);
          EXPECT_EQ(kTestParamVelocity5, test_node_->output_velocity().linear.y);
          EXPECT_EQ(kTestParamVelocity0, test_node_->output_velocity().angular.z);
          break;
        }
        loop_count++;
      }
    }

    // Store current value
    ASSERT_FALSE(test_node_->CheckTimeOut(start_time));
    // For input_velocity6 controlling only the rotational axis with high priority
    pre_data = test_node_->output_velocity().linear.x;
    test_node_->SpinOnce();
    rate.sleep();
  }
}

// and input_velocity0 controlling all axes with low priority
// Test whether the rotational axis of output_velocity stabilizes at input_velocity6
// and the XY axis stabilizes at input_velocity0
// Set flag ON when published value starts changing
TEST_F(VelocitySwitcherNodeTest, Control_specific_axis1) {
  rclcpp::Rate rate(kTestRate);
  const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();

  bool subscribed_data_changed = false;
  double pre_data = test_node_->output_velocity().linear.x;
  int32_t loop_count = 0;

  while (rclcpp::ok()) {
    PubTopic(kTestPatternEnum10);

    // If flag is ON, the value has started moving
    if (!subscribed_data_changed &&
        fabs(pre_data - test_node_->output_velocity().linear.x) > kCompareThreshold) {
      subscribed_data_changed = true;
    }

    // Previous and current values are the same
    if (subscribed_data_changed) {
      // Values remain the same for a certain period
      if (fabs(pre_data - test_node_->output_velocity().linear.x) < kCompareThreshold) {
        // Consider the value stable and perform the test
        if (loop_count > kTestParamLoopCountThresholdToCheckConstant) {
          // Timeout check
          EXPECT_EQ(kTestParamVelocity0, test_node_->output_velocity().linear.x);
          EXPECT_EQ(kTestParamVelocity0, test_node_->output_velocity().linear.y);
          EXPECT_EQ(kTestParamVelocity6, test_node_->output_velocity().angular.z);
          break;
        }
        loop_count++;
      }
    }

    // Store current value
    ASSERT_FALSE(test_node_->CheckTimeOut(start_time));
    // Confirm that no output occurs if no input velocity is provided
    pre_data = test_node_->output_velocity().linear.x;
    test_node_->SpinOnce();
    rate.sleep();
  }
}

// Confirm that output stops after input velocity ceases and stabilizes at 0
TEST_F(VelocitySwitcherNodeTest, No_input_velocity) {
  ASSERT_FALSE(test_node_->WaitForSubscriberReceived());
}

// Create velocity_switcher node
TEST_F(VelocitySwitcherNodeTest, Stop_output_velocity) {
  PubTopic(kTestPatternEnum0);
  ASSERT_TRUE(test_node_->WaitForSubscriberReceived());
  test_node_->WaitUntilOutputGetsZero();
  test_node_->ClearSubscribedFlag();
  ASSERT_FALSE(test_node_->WaitForSubscriberReceived());
}
}  // namespace tmc_velocity_switcher


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);
  const std::string yaml_directory = ament_index_cpp::get_package_share_directory("tmc_velocity_switcher")
      + "/test/parameter/";
  // Read parameters from yaml
  auto velocity_switcher_node = std::make_shared<tmc_velocity_switcher::VelocitySwitcher>(option);
  // Start a thread to spin
  LoadParameterFromYaml(velocity_switcher_node, yaml_directory, "velocity_switcher-test.yaml");
  velocity_switcher_node->Init();
  // Create a thread to spin
  auto velocity_switcher_node_thread = std::make_shared<std::thread>([&]() {
      rclcpp::spin(velocity_switcher_node);
      });
  testing::InitGoogleTest(&argc, argv);

  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  velocity_switcher_node_thread->join();
  velocity_switcher_node_thread.reset();

  return result;
}
