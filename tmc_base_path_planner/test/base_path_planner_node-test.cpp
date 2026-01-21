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
#include <chrono>
#include <limits>
#include <memory>
#include <optional>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tmc_base_path_planner/base_path_planner_node.hpp>
#include <tmc_navigation_msgs/action/base_path_plan.hpp>
#include <tmc_navigation_msgs/action/path_follower.hpp>
#include "cyclic_sender.hpp"
#include "path_follower_dummy.hpp"
#include "test_utils_ros.hpp"

namespace {
constexpr double kRate = 10.0;
constexpr double kTimeout = 5.0;
}  // anonymous namespace

namespace tmc_base_path_planner {
using PathPlanAction = tmc_navigation_msgs::action::BasePathPlan;
using PathPlanGoalHandle = rclcpp_action::ClientGoalHandle<PathPlanAction>;
using std::placeholders::_1;
using std::placeholders::_2;

/// Test node
class TestNode : public rclcpp::Node {
 public:
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options) {}

  void Init() {
    rate_ = std::make_shared<rclcpp::Rate>(kRate);
    planner_action_client_ = rclcpp_action::create_client<PathPlanAction>(this, "base_path_plan");
    pub_static_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "static_obstacle_ros_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    // Wait until linked with subscriber
    const rclcpp::Time start = this->get_clock()->now();
    while (pub_static_map_->get_subscription_count() == 0) {
      if (this->get_clock()->now() - start > rclcpp::Duration::from_seconds(10.0)) {
        RCLCPP_FATAL(this->get_logger(), "Can not link to subscriber.");
        exit(EXIT_FAILURE);
      }
      rate_->sleep();
    }
  }

  // Send static map
  void PublishStaticMap(nav_msgs::msg::OccupancyGrid& static_map) {
    static_map.header.stamp = this->get_clock()->now();
    pub_static_map_->publish(static_map);
  }

  // Send path planning action
  void SendPathPlanAction(const geometry_msgs::msg::PoseStamped& goal_pose) {
    auto send_goal_options = rclcpp_action::Client<PathPlanAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&TestNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&TestNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&TestNode::result_callback, this, _1);

    auto planner_action_goal = PathPlanAction::Goal();
    planner_action_goal.goal = goal_pose;
    planner_result_ = std::nullopt;
    planner_action_client_->async_send_goal(planner_action_goal, send_goal_options);
  }

  bool CancelPathPlanAction() {
    planner_action_client_->async_cancel_all_goals();
    return WaitUntil([&]() {
        return IsMatchActionResult(rclcpp_action::ResultCode::CANCELED,
        PathPlanAction::Result::CANCELED);
        }, kTimeout);
  }

  bool WaitForActionServer() {
    return planner_action_client_->wait_for_action_server(std::chrono::seconds(5));
  }

  // Determine if the result of the path planning action matches the argument
  bool IsMatchActionResult(const rclcpp_action::ResultCode& in_result_code,
                           const uint32_t in_reason) {
    if (!planner_result_) {
      return false;
    }
    if (planner_result_.value().code == in_result_code && planner_result_.value().result->reason == in_reason) {
      return true;
    }
    return false;
  }

  // Determine if the feedback of the path planning matches the argument
  bool IsMatchActionFeedback(const PathPlanAction::Feedback& in_feedback) {
    if (current_feedback_.status == in_feedback.status &&
        current_feedback_.reason == in_feedback.reason) {
      return true;
    }
    return false;
  }

  // Wait until some condition is met
  bool WaitUntil(std::function<bool()> condition_function, double timeout_sec) {
    // Error check of argument
    if (!condition_function) {
      throw std::invalid_argument("Function for waiting is empty.");
    }
    if (timeout_sec < 0.0) {
      throw std::invalid_argument("Timeout must must have fully value");
    }

    const rclcpp::Time end_time = this->get_clock()->now() + rclcpp::Duration::from_seconds(timeout_sec);
    while (rclcpp::ok()) {
      SpinOnce();
      if (condition_function()) return true;
      if (this->get_clock()->now() >= end_time) break;
    }
    return false;
  }

  void SpinOnce() {
    rclcpp::spin_some(shared_from_this());
    rate_->sleep();
  }

 private:
  // // Path planning action client callback
  void goal_response_callback(const PathPlanGoalHandle::SharedPtr& future) {}
  void feedback_callback(PathPlanGoalHandle::SharedPtr,
      const std::shared_ptr<const PathPlanAction::Feedback> feedback) {
    current_feedback_ = *feedback;
  }
  void result_callback(const PathPlanGoalHandle::WrappedResult& result) {
    planner_result_ = result;
  }

  std::shared_ptr<rclcpp::Rate> rate_;
  /// Path planning action client
  rclcpp_action::Client<PathPlanAction>::SharedPtr planner_action_client_;
  std::optional<PathPlanGoalHandle::WrappedResult> planner_result_;

  PathPlanAction::Feedback current_feedback_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_static_map_;
};


class BasePathPlannerNodeTest : public testing::Test {
 public:
  BasePathPlannerNodeTest() {}

  virtual ~BasePathPlannerNodeTest() = default;

 protected:
  virtual void SetUp() {
    // Set default data for testing
    // Generate map
    static_map_ = CreateFreeMap(300, 300, 0.05);
    dynamic_map_ = CreateFreeMap(100, 100, 0.05);
    // Pre-place walls on the static map
    static_map_wall_point_.x = 10.0;
    static_map_wall_point_.y = 10.0;
    DrawObstacleCircle(static_map_, static_map_wall_point_, 0.5);

    // Self-position
    global_pose_.header.frame_id = "map";
    global_pose_.pose = CreatePose(1.0, 1.0, 0.0);
    // Goal position
    goal_pose_.header.frame_id = "map";
    goal_pose_.pose = CreatePose(4.0, 4.0, 0.0);

    rate_ = std::make_shared<rclcpp::Rate>(kRate);

    // Generate target node
    // TODO(syuuhei_shiro): 下記、原因調査
    // Wanted to launch the target node only once in main, but
    // Even if Feedback is published, it is not notified to the callback of the test node
    // It was notified if regenerated for each setup, so use this method for now
    // In actual operation, the phenomenon of stopping Feedback notification has not occurred
    rclcpp::NodeOptions option;
    option.allow_undeclared_parameters();
    option.automatically_declare_parameters_from_overrides(true);
    base_path_planner_node_ = std::make_shared<BasePathPlannerNode>(option);
    const std::string yaml_directory =
        ament_index_cpp::get_package_share_directory("tmc_base_path_planner") + "/test/parameter/";
    LoadParameterFromYaml(base_path_planner_node_, yaml_directory, "base_path_planner_node-test.yaml");
    base_path_planner_node_->Init();
    base_path_planner_node_thread_ = std::make_shared<std::thread>([&]() {
        base_path_planner_node_thread_killed_ = false;
        while (rclcpp::ok() && !base_path_planner_node_thread_killed_) {
          rate_->sleep();
          rclcpp::spin_some(base_path_planner_node_);
        }
        });

    test_node_ = std::make_shared<TestNode>();
    cyclic_sender_ = std::make_shared<CyclicSender>();
    follower_dummy_ = std::make_shared<PathFollowerDummy>();

    test_node_->Init();
    cyclic_sender_->Init(kRate);
    follower_dummy_->Init(kRate);

    // Issue static map
    test_node_->PublishStaticMap(static_map_);

    // Start periodic transmission of dynamic map and self-position
    cyclic_sender_->StartSendDynamicMap(dynamic_map_);
    cyclic_sender_->StartSendGlobalPose(global_pose_);

    // Thread follower_dummy, cyclic_sender
    follower_dummy_thread_ = std::make_shared<std::thread>([&]() { follower_dummy_->Run(); });
    cyclic_sender_thread_ = std::make_shared<std::thread>([&]() { cyclic_sender_->Run(); });
    // Wait for the action server to launch
    if (!test_node_->WaitForActionServer()) {
      RCLCPP_FATAL(test_node_->get_logger(), "Path plan action was not established.");
      exit(EXIT_FAILURE);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  virtual void TearDown() {
    base_path_planner_node_thread_killed_ = true;
    follower_dummy_->Kill();
    cyclic_sender_->Kill();
    base_path_planner_node_thread_->join();
    follower_dummy_thread_->join();
    cyclic_sender_thread_->join();
  }


  std::shared_ptr<BasePathPlannerNode> base_path_planner_node_;
  std::shared_ptr<TestNode> test_node_;
  std::shared_ptr<CyclicSender> cyclic_sender_;
  std::shared_ptr<PathFollowerDummy> follower_dummy_;

  bool base_path_planner_node_thread_killed_;
  std::shared_ptr<std::thread> base_path_planner_node_thread_;
  std::shared_ptr<std::thread> follower_dummy_thread_;
  std::shared_ptr<std::thread> cyclic_sender_thread_;

  nav_msgs::msg::OccupancyGrid static_map_;
  nav_msgs::msg::OccupancyGrid dynamic_map_;
  geometry_msgs::msg::PoseStamped global_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  geometry_msgs::msg::Point static_map_wall_point_;
  std::shared_ptr<rclcpp::Rate> rate_;
};

/// Plan a path for the given goal
/// When Follower becomes SUCCEEDED, it becomes SUCCEEDED(REACHED)
TEST_F(BasePathPlannerNodeTest, PlanPath) {
  // Follower dummy is set to succeed after 1 second
  follower_dummy_->SetActionCompleteCondition(1.0, rclcpp_action::ResultCode::SUCCEEDED);

  // Execute path planning action
  test_node_->SendPathPlanAction(goal_pose_);

  // Become SUCCEEDED(REACHED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::SUCCEEDED,
      PathPlanAction::Result::REACHED);
      }, kTimeout));
  /// The final point of the path matches the input goal
  /// Node test does not consider whether the generated path points are valid
  EXPECT_TRUE(IsMatchPoseStamped(goal_pose_, follower_dummy_->CurrentRequestedPath().poses.back()));
}

/// If a dynamic obstacle is placed on the published path, a path that avoids it will be issued
TEST_F(BasePathPlannerNodeTest, ReplanPath) {
  // Execute path planning action
  test_node_->SendPathPlanAction(goal_pose_);
  rclcpp::Time time = test_node_->get_clock()->now();
  geometry_msgs::msg::Point obstacle_point;

  // When the first request comes to the follower, place a dynamic obstacle in the middle of the path
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout));
  const uint32_t index = static_cast<uint32_t>(follower_dummy_->CurrentRequestedPath().poses.size() / 2);
  obstacle_point = follower_dummy_->CurrentRequestedPath().poses[index].pose.position;
  DrawObstacleCircle(dynamic_map_, obstacle_point, 0.5);
  cyclic_sender_->StartSendDynamicMap(dynamic_map_);

  double distance_to_obstacle = 0.0;
  while (test_node_->get_clock()->now() - time < rclcpp::Duration::from_seconds(kTimeout)) {
    if (test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout)) {
      distance_to_obstacle = DistancePointToPath(obstacle_point, follower_dummy_->CurrentRequestedPath());
      if (distance_to_obstacle > std::numeric_limits<double>::epsilon()) {
        // Exit when the path is updated
        break;
      }
    }
  }
  test_node_->CancelPathPlanAction();
  // A path that avoids is issued
  EXPECT_GT(distance_to_obstacle, 0.5);
  // The final point of the path matches the input goal
  EXPECT_TRUE(IsMatchPoseStamped(goal_pose_, follower_dummy_->CurrentRequestedPath().poses.back()));
}


/// When a cancel is requested, it becomes CANCELED(CANCELED)
/// Cancel the Follower
TEST_F(BasePathPlannerNodeTest, Cancel) {
  // Execute path planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // Cancel when a request comes to the follower
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout));
  // Result becomes CANCELED(CANCELED) (checked within CancelPathPlanAction)
  EXPECT_TRUE(test_node_->CancelPathPlanAction());
  // Cancel the Follower
  EXPECT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsCanceled(); }, kTimeout));
}


/// Path planning is done according to the specified FrameID
TEST_F(BasePathPlannerNodeTest, TransformGoal) {
  geometry_msgs::msg::Pose map_child = CreatePose(1.0, 1.0, 0.0);
  cyclic_sender_->StartSendTransform("map", "map_child", map_child);
  // Execute path planning action
  goal_pose_.header.frame_id = "map_child";
  test_node_->SendPathPlanAction(goal_pose_);
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout));
  test_node_->CancelPathPlanAction();
  // The goal of the planned path is the coordinates transformed from map_child to map of the input goal
  geometry_msgs::msg::PoseStamped expect_pose = goal_pose_;
  expect_pose.pose.position.x = expect_pose.pose.position.x + map_child.position.x;
  expect_pose.pose.position.y = expect_pose.pose.position.y + map_child.position.y;
  EXPECT_EQ("map", follower_dummy_->CurrentRequestedPath().poses.back().header.frame_id);
  EXPECT_TRUE(IsMatchPoseStamped(expect_pose, follower_dummy_->CurrentRequestedPath().poses.back()));
}


/// If the specified FrameID does not exist, it becomes ABORTED(TRANSFORM_GOAL_ERROR)
TEST_F(BasePathPlannerNodeTest, TransformGoalError) {
  // Specify a non-existent FrameID
  goal_pose_.header.frame_id = "unknown";
  test_node_->SendPathPlanAction(goal_pose_);
  // Become Aborted(TRANSFORM_GOAL_ERROR)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::TRANSFORM_GOAL_ERROR);
      }, kTimeout));
}

// Disable test due to possible test failure in CodeBuild
// TODO(kazuki_shibamiya) : CodeBuildで安定的にテストが通るようにする
#if 0
/// When an action is thrown by overwrite, the preceding action ends with ABORTED(PREEMPTED)
/// The subsequent action becomes SUCCEEDED(REACHED)
/// Do not request cancel to Follower
TEST_F(BasePathPlannerNodeTest, NewGoalAvailable) {
  // Set completion time to avoid timeout until test ends
  follower_dummy_->SetActionCompleteCondition(kTimeout * 2.0, rclcpp_action::ResultCode::SUCCEEDED);
  // Execute path planning action
  test_node_->SendPathPlanAction(goal_pose_);

  // Execute path planning action again after a request occurs to the follower
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout));
  test_node_->SendPathPlanAction(goal_pose_);

  // The preceding action ends with ABORTED(PREEMPTED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::PREEMPTED);
      }, kTimeout));
  // No cancel request has come to the Follower
  EXPECT_FALSE(test_node_->WaitUntil([&]() { return follower_dummy_->IsCanceled(); }, kTimeout));

  // The subsequent action becomes SUCCEEDED(REACHED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::SUCCEEDED,
      PathPlanAction::Result::REACHED);
      }, kTimeout));
}
#endif

/// If the goal of the subsequent action is not plannable, even though the action was thrown by overwrite,
/// Request cancel to Follower
TEST_F(BasePathPlannerNodeTest, NewGoalAvailableFail) {
  // Execute path planning action
  test_node_->SendPathPlanAction(goal_pose_);

  // Execute path planning action again after a request occurs to the follower
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout));
  // Place the goal outside the range of the static map
  goal_pose_.pose = CreatePose(-1.0, -1.0, 0.0);
  test_node_->SendPathPlanAction(goal_pose_);

  // End with ABORTED(PREEMPTED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::PREEMPTED);
      }, kTimeout));
  // Request cancel to Follower
  EXPECT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsCanceled(); }, kTimeout));
}


/// If the goal is filled with obstacles on the dynamic map, path planning can be done when far from the goal, but fails when approaching the goal
/// Stop the Follower and notify Feedback with PLANNING(GOAL_IS_ON_DYNAMIC_OBSTACLE)
TEST_F(BasePathPlannerNodeTest, GoalIsOnDynamicObstacle) {
  // Get filter range around the goal
  double map_filter_range_around_goal;
  rclcpp::Parameter map_filter_range_around_goal_param;
  base_path_planner_node_->get_parameter("base_path_planner.map_filter.map_filter_range_around_goal",
      map_filter_range_around_goal_param);
  map_filter_range_around_goal = map_filter_range_around_goal_param.as_double();
  // Get filter distance around the goal
  double map_filter_distance_goal_limit;
  rclcpp::Parameter map_filter_distance_goal_limit_param;
  base_path_planner_node_->get_parameter("base_path_planner.map_filter.map_filter_distance_goal_limit",
      map_filter_distance_goal_limit_param);
  map_filter_distance_goal_limit = map_filter_distance_goal_limit_param.as_double();
  // Place an obstacle at the goal position with half the size of the filter range
  DrawObstacleCircle(dynamic_map_, goal_pose_.pose.position, map_filter_range_around_goal / 2.0);
  cyclic_sender_->StartSendDynamicMap(dynamic_map_);
  // Execute path planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // Successfully plan the path when far from the goal and request to Follower
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout));

  // Move self-position closer to the goal
  global_pose_.pose.position.x = goal_pose_.pose.position.x + map_filter_distance_goal_limit - 0.1;
  global_pose_.pose.position.y = goal_pose_.pose.position.y;
  cyclic_sender_->StartSendGlobalPose(global_pose_);

  // PLANNING(GOAL_IS_ON_DYNAMIC_OBSTACLE) is fed back
  PathPlanAction::Feedback expect_feedback;
  expect_feedback.status = PathPlanAction::Feedback::PLANNING;
  expect_feedback.reason = PathPlanAction::Feedback::GOAL_IS_ON_DYNAMIC_OBSTACLE;
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionFeedback(expect_feedback);
    }, kTimeout));

  // Request cancel to Follower
  EXPECT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsCanceled(); }, kTimeout));
  test_node_->CancelPathPlanAction();
}


/// If the path to the goal is completely blocked by obstacles on the dynamic map,
/// Stop the Follower and notify Feedback with PLANNING(PATH_PLANNING_FAIL)
TEST_F(BasePathPlannerNodeTest, PathPlanningFail) {
  // Get filter range around the goal
  double map_filter_range_around_goal;
  rclcpp::Parameter map_filter_range_around_goal_param;
  base_path_planner_node_->get_parameter("base_path_planner.map_filter.map_filter_range_around_goal",
      map_filter_range_around_goal_param);
  map_filter_range_around_goal = map_filter_range_around_goal_param.as_double();

  // Execute path planning action
  test_node_->SendPathPlanAction(goal_pose_);
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout));

  // Place an obstacle at the goal position slightly larger than the filter range around the goal to fill the surroundings with obstacles
  DrawObstacleCircle(dynamic_map_, goal_pose_.pose.position, map_filter_range_around_goal + 0.1);
  cyclic_sender_->StartSendDynamicMap(dynamic_map_);

  // PLANNING(PATH_PLANNING_FAIL) is fed back
  PathPlanAction::Feedback expect_feedback;
  expect_feedback.status = PathPlanAction::Feedback::PLANNING;
  expect_feedback.reason = PathPlanAction::Feedback::PATH_PLANNING_FAIL;
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionFeedback(expect_feedback);
    }, kTimeout));
  // Request cancel to Follower
  EXPECT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsCanceled(); }, kTimeout));
  test_node_->CancelPathPlanAction();
}


/// If the goal is on the wall of the static map, it becomes ABORTED(GOAL_IS_ON_STATIC_OBSTACLE)
TEST_F(BasePathPlannerNodeTest, GoalIsOnStaticObstacle) {
  // Place the goal position on the wall of the static map
  goal_pose_.pose.position = static_map_wall_point_;
  // Execute path planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // End with ABORTED(GOAL_IS_ON_STATIC_OBSTACLE)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::GOAL_IS_ON_STATIC_OBSTACLE);
    }, kTimeout));
}


/// If the goal is outside the range of the static map, it becomes ABORTED(GOAL_IS_ON_STATIC_OBSTACLE)
TEST_F(BasePathPlannerNodeTest, GoalIsOutOfMap) {
  // Place the goal position outside the range of the static map
  goal_pose_.pose = CreatePose(-1.0, -1.0, 0.0);
  // Execute path planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // End with ABORTED(GOAL_IS_ON_STATIC_OBSTACLE)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::GOAL_IS_ON_STATIC_OBSTACLE);
    }, kTimeout));
}


/// If the robot position is outside the range of the static map, it becomes ABORTED(ROBOT_IS_OUT_OF_MAP)
TEST_F(BasePathPlannerNodeTest, RobotIsOutOfMap) {
  // Place the robot position outside the range of the static map
  global_pose_.pose = CreatePose(-1.0, -1.0, 0.0);
  cyclic_sender_->StartSendGlobalPose(global_pose_);
  // Execute path planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // End with ABORTED(ROBOT_IS_OUT_OF_MAP)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::ROBOT_IS_OUT_OF_MAP);
    }, kTimeout));
}

/// When Follower becomes ABORTED, it becomes ABORTED(FOLLOWER_ABORTED)
TEST_F(BasePathPlannerNodeTest, FollowerAborted) {
  // Follower dummy is set to become ABORTED after 1 second
  follower_dummy_->SetActionCompleteCondition(1.0, rclcpp_action::ResultCode::ABORTED);
  // Execute path planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // Become ABORTED(FOLLOWER_ABORTED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::FOLLOWER_ABORTED);
      }, kTimeout));
}

/// When the dynamic map times out, it becomes ABORTED(DYNAMIC_MAP_IS_NOT_UPDATED)
TEST_F(BasePathPlannerNodeTest, DynamicMapIsNotUpdated) {
  // Stop issuing the dynamic map
  cyclic_sender_->StopSendDynamicMap();
  // Execute path planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // Become ABORTED(DYNAMIC_MAP_IS_NOT_UPDATED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::DYNAMIC_MAP_IS_NOT_UPDATED);
    }, kTimeout));
}


/// When self-position times out, it becomes ABORTED(ROBOT_POSE_IS_NOT_UPDATED)
TEST_F(BasePathPlannerNodeTest, RobotPoseIsNotUpdated) {
  // Stop issuing self-position
  cyclic_sender_->StopSendGlobalPose();
  // Execute path planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // Become ABORTED(ROBOT_POSE_IS_NOT_UPDATED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::ROBOT_POSE_IS_NOT_UPDATED);
    }, kTimeout));
}
}  // namespace tmc_base_path_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
