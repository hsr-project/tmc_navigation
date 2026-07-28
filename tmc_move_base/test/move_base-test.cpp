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
/// @file move_base-test.cpp
/// @brief Test for autonomous navigation action node
#include <chrono>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tmc_move_base/move_base.hpp>
#include <tmc_navigation_msgs/action/base_path_plan.hpp>
#include "path_planner_dummy.hpp"
#include "test_utils.hpp"

namespace tmc_move_base {
using std::placeholders::_1;
using std::placeholders::_2;
using MoveBaseClientGoalHandle = rclcpp_action::ClientGoalHandle<MoveBaseAction>;

constexpr const char* const kMapFrameName = "map";
constexpr const char* const kOtherFrameName = "other_frame";
constexpr double kPlanningTimeoutParamValue = 10.0;
constexpr double kTimeout = 5.0;
constexpr double kRate = 10.0;

geometry_msgs::msg::Transform CreateTransform(const double x, const double y, const double yaw) {
  geometry_msgs::msg::Transform transform;
  transform.translation.x = x;
  transform.translation.y = y;
  transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  transform.rotation.x = q.x();
  transform.rotation.y = q.y();
  transform.rotation.z = q.z();
  transform.rotation.w = q.w();
  return transform;
}

/// Test node
class TestNode : public rclcpp::Node {
 public:
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options) {}

  void Init() {
    rate_ = std::make_shared<rclcpp::Rate>(kRate);
    action_client_ = rclcpp_action::create_client<MoveBaseAction>(this, "move_base/move");
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("move_base_simple/goal", 1);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // Wait until linked with the subscriber
    if (!WaitUntil([&]() { return (goal_publisher_->get_subscription_count() != 0); }, kTimeout)) {
      RCLCPP_FATAL(rclcpp::get_logger("move_base-test"), "Can not link to subscriber.");
      exit(EXIT_FAILURE);
    }
    // Wait for the action server to start
    if (!action_client_->wait_for_action_server(std::chrono::seconds(20))) {
      RCLCPP_FATAL(rclcpp::get_logger("move_base-test"), "Move Base Action was not established.");
      exit(EXIT_FAILURE);
    }
  }

  // Send the goal topic
  void SendGoalTopic(const geometry_msgs::msg::PoseStamped& goal_pose) {
    geometry_msgs::msg::PoseStamped goal = goal_pose;
    goal.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    goal_publisher_->publish(goal);
  }

  // Send the action goal
  void SendMoveBaseActionGoal(const geometry_msgs::msg::PoseStamped& goal_pose) {
    auto send_goal_options = rclcpp_action::Client<MoveBaseAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&TestNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&TestNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&TestNode::result_callback, this, _1);

    auto goal = MoveBaseAction::Goal();
    goal.pose = goal_pose;
    goal.pose.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    move_base_result_ = std::nullopt;
    action_client_->async_send_goal(goal, send_goal_options);
  }

  // Determine if the action result matches the argument
  bool IsMatchActionResult(const rclcpp_action::ResultCode& in_result_code) {
    if (move_base_result_ && move_base_result_.value().code == in_result_code) {
      return true;
    }
    return false;
  }

  // Cancel the action
  bool CancelMoveBaseAction() {
    action_client_->async_cancel_all_goals();
    return WaitUntil([&]() {
      return IsMatchActionResult(rclcpp_action::ResultCode::CANCELED);
      }, kTimeout);
  }

  // Send StaticTF
  void SendStaticTransform(const std::string& frame, const std::string& child_frame,
      const geometry_msgs::msg::Transform& transform) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = frame;
    transform_stamped.child_frame_id = child_frame;
    transform_stamped.transform = transform;
    tf_static_broadcaster_->sendTransform(transform_stamped);
  }

  // Wait until some condition is met
  bool WaitUntil(std::function<bool()> condition_function, double timeout_sec) {
    // Error check for arguments
    if (!condition_function) {
      throw std::invalid_argument("Function for waiting is empty.");
    }
    if (timeout_sec < 0.0) {
      throw std::invalid_argument("Timeout must must have fully value");
    }

    const rclcpp::Time end_time = rclcpp::Clock(RCL_ROS_TIME).now() + rclcpp::Duration::from_seconds(timeout_sec);
    while (rclcpp::ok()) {
      SpinOnce();
      if (condition_function()) return true;
      if (rclcpp::Clock(RCL_ROS_TIME).now() >= end_time) break;
    }
    return false;
  }

  void SpinOnce() {
    rclcpp::spin_some(shared_from_this());
    rate_->sleep();
  }

 private:
  void goal_response_callback(const MoveBaseClientGoalHandle::SharedPtr& future) {}
  void feedback_callback(MoveBaseClientGoalHandle::SharedPtr,
      const std::shared_ptr<const MoveBaseAction::Feedback> feedback) {}
  void result_callback(const MoveBaseClientGoalHandle::WrappedResult& result) {
    move_base_result_ = result;
  }

  std::shared_ptr<rclcpp::Rate> rate_;
  std::optional<MoveBaseClientGoalHandle::WrappedResult> move_base_result_;
  // move_base action client
  rclcpp_action::Client<MoveBaseAction>::SharedPtr action_client_;
  // goal topic publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  // StaticTF Broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

// Test fixture
class MoveBaseTest : public ::testing::Test {
 public:
  MoveBaseTest() {}
  ~MoveBaseTest() {}

 protected:
  virtual void SetUp() {
    // Generate planner_dummy node
    path_planner_dummy_ = std::make_shared<PathPlannerDummy>();
    path_planner_dummy_->Init(kRate);

    // Thread the planner_dummy node
    path_planner_dummy_thread_ = std::make_shared<std::thread>([&]() { path_planner_dummy_->Run(); });
    // Generate test node
    test_node_ = std::make_shared<TestNode>();
    test_node_->Init();

    // Publish StaticTF
    test_node_->SendStaticTransform(kMapFrameName, kOtherFrameName, CreateTransform(10.0, 10.0, 1.57));
    /// Set goal position
    /// Set two PoseStamped to match after coordinate transformation
    goal_in_map_frame_.header.frame_id = kMapFrameName;
    goal_in_map_frame_.pose = CreatePose(10.0, 11.0, 3.14);
    goal_in_other_frame_.header.frame_id = kOtherFrameName;
    goal_in_other_frame_.pose = CreatePose(1.0, 0.0, 1.57);
  }

  virtual void TearDown() {
    path_planner_dummy_->Kill();
    path_planner_dummy_thread_->join();
  }

  std::shared_ptr<MoveBase> move_base_node_;
  std::shared_ptr<TestNode> test_node_;
  std::shared_ptr<PathPlannerDummy> path_planner_dummy_;

  std::shared_ptr<std::thread> path_planner_dummy_thread_;

  // Goal coordinates based on map frame
  geometry_msgs::msg::PoseStamped goal_in_map_frame_;
  // Goal coordinates based on other frame
  geometry_msgs::msg::PoseStamped goal_in_other_frame_;
};


/// When autonomous navigation is requested via action, request route planning from PathPlanner
/// When PathPlanner becomes SUCCEEDED, it should also become SUCCEEDED
TEST_F(MoveBaseTest, RequestByAction) {
  // PathPlanner dummy is set to become SUCCEEDED after 1 second
  path_planner_dummy_->SetActionCompleteCondition(1.0, rclcpp_action::ResultCode::SUCCEEDED);
  // Execute action
  test_node_->SendMoveBaseActionGoal(goal_in_map_frame_);

  // Route planning should be requested
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return path_planner_dummy_->IsRequested(); }, kTimeout));

  // Should become SUCCEEDED
  ASSERT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::SUCCEEDED); }, kTimeout));

  // Route planning should be requested based on the input goal coordinates
  EXPECT_TRUE(IsMatchPoseStamped(path_planner_dummy_->current_requested_goal(), goal_in_map_frame_));
}


/// When cancellation is requested, it should become CANCELED
/// Cancel PathPlanner
TEST_F(MoveBaseTest, Cancel) {
  // Execute action
  test_node_->SendMoveBaseActionGoal(goal_in_map_frame_);
  // Cancel when route planning is requested
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return path_planner_dummy_->IsRequested(); }, kTimeout));
  // Should become CANCELED (checked within CancelMoveBaseAction)
  EXPECT_TRUE(test_node_->CancelMoveBaseAction());
  // Cancel PathPlanner
  EXPECT_TRUE(test_node_->WaitUntil([&]() { return path_planner_dummy_->IsCanceled(); }, kTimeout));
}


/// Convert the specified FrameID to map coordinates and request route planning
TEST_F(MoveBaseTest, TransformGoal) {
  // Specify goal based on other frame and execute action
  test_node_->SendMoveBaseActionGoal(goal_in_other_frame_);

  // Route planning should be requested
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return path_planner_dummy_->IsRequested(); }, kTimeout));

  // Should become SUCCEEDED
  ASSERT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::SUCCEEDED); }, kTimeout));

  // Convert input goal coordinates to map coordinates and request route planning
  EXPECT_TRUE(IsMatchPoseStamped(path_planner_dummy_->current_requested_goal(), goal_in_map_frame_));
}


/// When the specified FrameID does not exist, it should become ABORTED
TEST_F(MoveBaseTest, TransformGoalError) {
  // Specify a non-existent FrameID and execute action
  geometry_msgs::msg::PoseStamped goal_in_unknown_frame = goal_in_other_frame_;
  goal_in_unknown_frame.header.frame_id = "unknown";
  test_node_->SendMoveBaseActionGoal(goal_in_unknown_frame);

  // Should become ABORTED
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED); }, kTimeout));
}


/// When an action is overridden, the preceding action should end with ABORTED
/// Do not request cancellation from PathPlanner
TEST_F(MoveBaseTest, NewGoalAvailable) {
  // PathPlanner dummy is set to succeed after 1 second
  path_planner_dummy_->SetActionCompleteCondition(1.0, rclcpp_action::ResultCode::SUCCEEDED);
  // Execute action
  test_node_->SendMoveBaseActionGoal(goal_in_map_frame_);

  // When a request is made to PathPlanner, request MoveBase action again
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return path_planner_dummy_->IsRequested(); }, kTimeout));

  test_node_->SendMoveBaseActionGoal(goal_in_other_frame_);

  // The preceding action should end with ABORTED
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED); }, kTimeout));
  // No cancellation request should be made to PathPlanner
  EXPECT_FALSE(path_planner_dummy_->IsCanceled());

  // The subsequent action should end with SUCCEEDED
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::SUCCEEDED); }, kTimeout));
}


/// When PathPlanner becomes ABORTED, it should also become ABORTED
TEST_F(MoveBaseTest, PathPlannerAborted) {
  // PathPlanner dummy is set to become Aborted after 1 second
  path_planner_dummy_->SetActionCompleteCondition(1.0, rclcpp_action::ResultCode::ABORTED);
  // Execute action
  test_node_->SendMoveBaseActionGoal(goal_in_map_frame_);
  // Should become ABORTED
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED); },
      kTimeout));
}


/// If PathPlanner remains in a failed state (PLANNING) for longer than the timeout period,
/// Stop route planning and become ABORTED
TEST_F(MoveBaseTest, PathPlanFailTimeout) {
  /// Set PathPlanner dummy to continuously feedback PLANNING
  /// Set to operate longer than the timeout period and become SUCCEEDED
  PathPlanAction::Feedback feedback;
  feedback.status = PathPlanAction::Feedback::PLANNING;
  feedback.reason = PathPlanAction::Feedback::PATH_PLANNING_FAIL;
  path_planner_dummy_->SetActionFeedback(feedback);
  path_planner_dummy_->SetActionCompleteCondition(kPlanningTimeoutParamValue + 1.0,
                                                  rclcpp_action::ResultCode::SUCCEEDED);

  // Execute action
  test_node_->SendMoveBaseActionGoal(goal_in_map_frame_);

  /// Should not become ABORTED until the timeout
  /// Due to concerns about unintended results caused by lag under heavy load in the execution environment with strict time settings,
  /// Confirm up to 2 seconds before the timeout with some margin
  ASSERT_FALSE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED); },
      kPlanningTimeoutParamValue - 2.0));
  // No cancellation request should be made to PathPlanner
  EXPECT_FALSE(path_planner_dummy_->IsCanceled());

  // Should become ABORTED after timeout
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED); },
      2.0 + kTimeout));
  // Request cancellation from PathPlanner
  EXPECT_TRUE(path_planner_dummy_->IsCanceled());
}


/// After PathPlanner remains in a failed state (PLANNING) for less than the timeout period, transition to route planning success (RUNNING)
/// Continue autonomous navigation, and when PathPlanner becomes SUCCEEDED, it should also become SUCCEEDED
TEST_F(MoveBaseTest, PathPlanFailRecovery) {
  /// Set PathPlanner dummy to continuously feedback PLANNING
  /// Set to operate longer than the timeout period and become SUCCEEDED
  PathPlanAction::Feedback feedback;
  feedback.status = PathPlanAction::Feedback::PLANNING;
  feedback.reason = PathPlanAction::Feedback::PATH_PLANNING_FAIL;
  path_planner_dummy_->SetActionFeedback(feedback);
  path_planner_dummy_->SetActionCompleteCondition(kPlanningTimeoutParamValue + 1.0,
                                                  rclcpp_action::ResultCode::SUCCEEDED);

  // Execute action
  test_node_->SendMoveBaseActionGoal(goal_in_map_frame_);

  /// Should not become ABORTED until the timeout
  /// Due to concerns about unintended results caused by lag under heavy load in the execution environment with strict time settings,
  /// Confirm up to 2 seconds before the timeout with some margin
  ASSERT_FALSE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED); },
      kPlanningTimeoutParamValue - 2.0));

  // Change PathPlanner dummy feedback to RUNNING
  feedback.status = PathPlanAction::Feedback::RUNNING;
  feedback.reason = PathPlanAction::Feedback::NONE;
  path_planner_dummy_->SetActionFeedback(feedback);

  // Should become SUCCEEDED
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::SUCCEEDED); },
      2.0 + kTimeout));
  // No cancellation request should be made to PathPlanner
  EXPECT_FALSE(path_planner_dummy_->IsCanceled());
}

/// When autonomous navigation is requested via topic, request route planning from PathPlanner
TEST_F(MoveBaseTest, RequestByTopic) {
  // Send goal topic
  test_node_->SendGoalTopic(goal_in_map_frame_);

  // Request route planning from PathPlanner
  EXPECT_TRUE(test_node_->WaitUntil([&]() { return path_planner_dummy_->IsRequested(); }, kTimeout));

  // Check if the input goal is commanded to PathPlanner
  EXPECT_TRUE(IsMatchPoseStamped(path_planner_dummy_->current_requested_goal(), goal_in_map_frame_));
  // Wait until the Follower action completes before ending the test to avoid anomalies
  EXPECT_TRUE(test_node_->WaitUntil([&]() { return !path_planner_dummy_->IsRunning(); }, kTimeout));
}
}  // namespace tmc_move_base

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  // Generate move_base node
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);
  auto move_base_node = std::make_shared<tmc_move_base::MoveBase>(option);
  // Set planning_timeout parameter
  rclcpp::Parameter planning_timeout_param("planning_timeout", tmc_move_base::kPlanningTimeoutParamValue);
  move_base_node->declare_parameter("planning_timeout", tmc_move_base::kPlanningTimeoutParamValue);
  move_base_node->set_parameter(planning_timeout_param);
  move_base_node->Init();
  // Create thread
  auto move_base_node_thread = std::make_shared<std::thread>([&]() {
      rclcpp::spin(move_base_node);
      });

  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  move_base_node_thread->join();
  move_base_node.reset();

  return result;
}
