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
constexpr double kRate = 20.0;
constexpr double kTimeout = 5.0;
}  // anonymous namespace

namespace tmc_base_path_planner {
using PathPlanAction = tmc_navigation_msgs::action::BasePathPlan;
using PathPlanGoalHandle = rclcpp_action::ClientGoalHandle<PathPlanAction>;
using std::placeholders::_1;
using std::placeholders::_2;

/// Test Node
class TestNode : public rclcpp::Node {
 public:
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options) {}

  void Init() {
    rate_ = std::make_shared<rclcpp::Rate>(kRate);
    planner_action_client_ = rclcpp_action::create_client<PathPlanAction>(this, "base_path_plan");
    pub_static_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "static_obstacle_ros_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    // Wait until linked with a subscriber
    const rclcpp::Time start = this->get_clock()->now();
    while (pub_static_map_->get_subscription_count() == 0) {
      if (this->get_clock()->now() - start > rclcpp::Duration::from_seconds(10.0)) {
        RCLCPP_FATAL(this->get_logger(), "Can not link to subscriber.");
        exit(EXIT_FAILURE);
      }
      rate_->sleep();
    }

    // Set parameter service
    rclcpp::AsyncParametersClient::SharedPtr param_client =
        std::make_shared<rclcpp::AsyncParametersClient>(this, "/base_path_planner");
    if (!param_client->wait_for_service(std::chrono::milliseconds(static_cast<int32_t>(kTimeout * 1000)))) {
      RCLCPP_FATAL(this->get_logger(), "Parameter service error.");
      exit(EXIT_FAILURE);
    }

    // Retrieve parameter
    std::vector<std::string> parameters_list = {"base_path_planner.map_filter.map_filter_range_around_goal",
                                                "base_path_planner.map_filter.map_filter_distance_goal_limit"};
    auto get_param_future = param_client->get_parameters(parameters_list);
    if (rclcpp::spin_until_future_complete(shared_from_this(), get_param_future,
        std::chrono::milliseconds(static_cast<int32_t>(kTimeout * 1000))) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_FATAL(get_logger(), "Failed to get parameter.");
      exit(EXIT_FAILURE);
    }
    std::vector<rclcpp::Parameter> responese_param = get_param_future.get();
    map_filter_range_around_goal_ = responese_param[0].as_double();
    map_filter_distance_goal_limit_ = responese_param[1].as_double();
  }

  // Send static map
  void PublishStaticMap(nav_msgs::msg::OccupancyGrid& static_map) {
    static_map.header.stamp = this->get_clock()->now();
    pub_static_map_->publish(static_map);
  }

  // Send route planning action
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

  // Determine if the result of the route planning action matches the argument
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

  // Determine if the feedback of the route planning matches the argument
  bool IsMatchActionFeedback(const PathPlanAction::Feedback& in_feedback) {
    if (current_feedback_.status == in_feedback.status &&
        current_feedback_.reason == in_feedback.reason) {
      return true;
    }
    return false;
  }

  // Wait until some condition is met
  bool WaitUntil(std::function<bool()> condition_function, double timeout_sec) {
    // Argument error check
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

  // Get filter range around the goal
  double map_filter_range_around_goal(void) const {
    return map_filter_range_around_goal_;
  }

  // Get filter distance around the goal
  double map_filter_distance_goal_limit(void) const {
    return map_filter_distance_goal_limit_;
  }

 private:
  // // Route planning action client callback
  void goal_response_callback(const PathPlanGoalHandle::SharedPtr& future) {}
  void feedback_callback(PathPlanGoalHandle::SharedPtr,
      const std::shared_ptr<const PathPlanAction::Feedback> feedback) {
    current_feedback_ = *feedback;
  }
  void result_callback(const PathPlanGoalHandle::WrappedResult& result) {
    planner_result_ = result;
  }

  std::shared_ptr<rclcpp::Rate> rate_;
  /// Route planning action client
  rclcpp_action::Client<PathPlanAction>::SharedPtr planner_action_client_;
  std::optional<PathPlanGoalHandle::WrappedResult> planner_result_;

  PathPlanAction::Feedback current_feedback_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_static_map_;

  double map_filter_range_around_goal_;
  double map_filter_distance_goal_limit_;
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

    test_node_ = std::make_shared<TestNode>();
    cyclic_sender_ = std::make_shared<CyclicSender>();
    follower_dummy_ = std::make_shared<PathFollowerDummy>();

    test_node_->Init();
    cyclic_sender_->Init(kRate);
    follower_dummy_->Init(kRate);

    // Publish static map
    test_node_->PublishStaticMap(static_map_);

    // Thread cyclic_sender
    cyclic_sender_thread_ = std::make_shared<std::thread>([&]() { cyclic_sender_->Run(); });

    // Start periodic transmission of dynamic map and self-position
    cyclic_sender_->StartSendDynamicMap(dynamic_map_);
    cyclic_sender_->StartSendGlobalPose(global_pose_);

    // Thread follower_dummy
    follower_dummy_thread_ = std::make_shared<std::thread>([&]() { follower_dummy_->Run(); });

    // Wait for the action server to start
    if (!test_node_->WaitForActionServer()) {
      RCLCPP_FATAL(test_node_->get_logger(), "Path plan action was not established.");
      exit(EXIT_FAILURE);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  virtual void TearDown() {
    follower_dummy_->Kill();
    cyclic_sender_->Kill();
    follower_dummy_thread_->join();
    cyclic_sender_thread_->join();
  }

  std::shared_ptr<TestNode> test_node_;
  std::shared_ptr<CyclicSender> cyclic_sender_;
  std::shared_ptr<PathFollowerDummy> follower_dummy_;

  std::shared_ptr<std::thread> follower_dummy_thread_;
  std::shared_ptr<std::thread> cyclic_sender_thread_;

  nav_msgs::msg::OccupancyGrid static_map_;
  nav_msgs::msg::OccupancyGrid dynamic_map_;
  geometry_msgs::msg::PoseStamped global_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  geometry_msgs::msg::Point static_map_wall_point_;
  std::shared_ptr<rclcpp::Rate> rate_;
};

/// Plan a route for the given goal
/// Becomes SUCCEEDED(REACHED) when the Follower becomes SUCCEEDED
TEST_F(BasePathPlannerNodeTest, PlanPath) {
  // Set follower dummy to succeed after 1 second
  follower_dummy_->SetActionCompleteCondition(1.0, rclcpp_action::ResultCode::SUCCEEDED);

  // Execute route planning action
  test_node_->SendPathPlanAction(goal_pose_);

  // Becomes SUCCEEDED(REACHED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::SUCCEEDED,
      PathPlanAction::Result::REACHED);
      }, kTimeout));
  /// The final point of the route matches the input goal
  /// Node tests do not consider whether the generated route points are valid
  EXPECT_TRUE(IsMatchPoseStamped(goal_pose_, follower_dummy_->CurrentRequestedPath().poses.back()));
}

/// If a dynamic obstacle is placed on the published route, a detour route is published
TEST_F(BasePathPlannerNodeTest, ReplanPath) {
  // Execute route planning action
  test_node_->SendPathPlanAction(goal_pose_);
  rclcpp::Time time = test_node_->get_clock()->now();
  geometry_msgs::msg::Point obstacle_point;

  // Place a dynamic obstacle on the route when the first request comes to the follower
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
        // Exit when the route is updated
        break;
      }
    }
  }
  test_node_->CancelPathPlanAction();
  // A detour route is published
  EXPECT_GT(distance_to_obstacle, 0.5);
  // The final point of the route matches the input goal
  EXPECT_TRUE(IsMatchPoseStamped(goal_pose_, follower_dummy_->CurrentRequestedPath().poses.back()));
}


/// Becomes CANCELED(CANCELED) when a cancel is requested
/// Cancel the Follower
TEST_F(BasePathPlannerNodeTest, Cancel) {
  // Execute route planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // Cancel when a request comes to the follower
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout));
  // Result becomes CANCELED(CANCELED) (checked within CancelPathPlanAction)
  EXPECT_TRUE(test_node_->CancelPathPlanAction());
  // Cancel the Follower
  EXPECT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsCanceled(); }, kTimeout));
}


/// Route planning is done according to the specified FrameID
TEST_F(BasePathPlannerNodeTest, TransformGoal) {
  geometry_msgs::msg::Pose map_child = CreatePose(1.0, 1.0, 0.0);
  cyclic_sender_->StartSendTransform("map", "map_child", map_child);
  // Execute route planning action
  goal_pose_.header.frame_id = "map_child";
  test_node_->SendPathPlanAction(goal_pose_);
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout));
  test_node_->CancelPathPlanAction();
  // The goal of the planned route is the coordinate transformed from map_child to map for the input goal
  geometry_msgs::msg::PoseStamped expect_pose = goal_pose_;
  expect_pose.pose.position.x = expect_pose.pose.position.x + map_child.position.x;
  expect_pose.pose.position.y = expect_pose.pose.position.y + map_child.position.y;
  EXPECT_EQ("map", follower_dummy_->CurrentRequestedPath().poses.back().header.frame_id);
  EXPECT_TRUE(IsMatchPoseStamped(expect_pose, follower_dummy_->CurrentRequestedPath().poses.back()));
}


/// Becomes ABORTED(TRANSFORM_GOAL_ERROR) if the specified FrameID does not exist
TEST_F(BasePathPlannerNodeTest, TransformGoalError) {
  // Specify a non-existent FrameID
  goal_pose_.header.frame_id = "unknown";
  test_node_->SendPathPlanAction(goal_pose_);
  // Becomes Aborted(TRANSFORM_GOAL_ERROR)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::TRANSFORM_GOAL_ERROR);
      }, kTimeout));
}

/// If an action is overridden, the preceding action ends with ABORTED(PREEMPTED)
/// The subsequent action becomes SUCCEEDED(REACHED)
/// Do not request cancel for the Follower
TEST_F(BasePathPlannerNodeTest, NewGoalAvailable) {
  // Set completion time to avoid timeout until the test ends
  follower_dummy_->SetActionCompleteCondition(kTimeout * 2.0, rclcpp_action::ResultCode::SUCCEEDED);
  // Execute route planning action
  test_node_->SendPathPlanAction(goal_pose_);

  // Execute route planning action again after a request is made to the follower
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout));
  test_node_->SendPathPlanAction(goal_pose_);

  // The preceding action ends with ABORTED(PREEMTED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::PREEMPTED);
      }, kTimeout));
  // No cancel request has been made to the Follower
  EXPECT_FALSE(test_node_->WaitUntil([&]() { return follower_dummy_->IsCanceled(); }, kTimeout));

  // The subsequent action becomes SUCCEEDED(REACHED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::SUCCEEDED,
      PathPlanAction::Result::REACHED);
      }, kTimeout));
}

/// If an action is overridden but the goal of the subsequent action is not plannable,
/// Request cancel for the Follower
TEST_F(BasePathPlannerNodeTest, NewGoalAvailableFail) {
  // Execute route planning action
  test_node_->SendPathPlanAction(goal_pose_);

  // Execute route planning action again after a request is made to the follower
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout));
  // Place the goal outside the range of the static map
  goal_pose_.pose = CreatePose(-1.0, -1.0, 0.0);
  test_node_->SendPathPlanAction(goal_pose_);

  // Ends with ABORTED(PREEMTED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::PREEMPTED);
      }, kTimeout));
  // Request cancel for the Follower
  EXPECT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsCanceled(); }, kTimeout));
}


/// If the goal is filled with obstacles in the dynamic map, route planning succeeds when far from the goal but fails when close to the goal
/// Stop the Follower and notify PLANNING(GOAL_IS_ON_DYNAMIC_OBSTACLE) in feedback
TEST_F(BasePathPlannerNodeTest, GoalIsOnDynamicObstacle) {
  // Place an obstacle at the goal position with half the size of the filter range
  DrawObstacleCircle(dynamic_map_, goal_pose_.pose.position, test_node_->map_filter_range_around_goal() / 2.0);
  cyclic_sender_->StartSendDynamicMap(dynamic_map_);
  // Execute route planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // Route planning succeeds when far from the goal and requests the Follower
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout));

  // Move self-position closer to the goal
  global_pose_.pose.position.x = goal_pose_.pose.position.x + test_node_->map_filter_distance_goal_limit() - 0.1;
  global_pose_.pose.position.y = goal_pose_.pose.position.y;
  cyclic_sender_->StartSendGlobalPose(global_pose_);

  // PLANNING(GOAL_IS_ON_DYNAMIC_OBSTACLE) is fed back
  PathPlanAction::Feedback expect_feedback;
  expect_feedback.status = PathPlanAction::Feedback::PLANNING;
  expect_feedback.reason = PathPlanAction::Feedback::GOAL_IS_ON_DYNAMIC_OBSTACLE;
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionFeedback(expect_feedback);
    }, kTimeout));

  // Request cancel for the Follower
  EXPECT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsCanceled(); }, kTimeout));
  test_node_->CancelPathPlanAction();
}


/// If the path to the goal is completely blocked by obstacles in the dynamic map,
/// Stop the Follower and notify PLANNING(PATH_PLANNING_FAIL) in feedback
TEST_F(BasePathPlannerNodeTest, PathPlanningFail) {
  // Execute route planning action
  test_node_->SendPathPlanAction(goal_pose_);
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsRequested(); }, kTimeout));

  // Place obstacles slightly larger than the filter range around the goal to fill the area around the goal
  DrawObstacleCircle(dynamic_map_, goal_pose_.pose.position, test_node_->map_filter_range_around_goal() + 0.1);
  cyclic_sender_->StartSendDynamicMap(dynamic_map_);

  // PLANNING(PATH_PLANNING_FAIL) is fed back
  PathPlanAction::Feedback expect_feedback;
  expect_feedback.status = PathPlanAction::Feedback::PLANNING;
  expect_feedback.reason = PathPlanAction::Feedback::PATH_PLANNING_FAIL;
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionFeedback(expect_feedback);
    }, kTimeout));
  // Request cancel for the Follower
  EXPECT_TRUE(test_node_->WaitUntil([&]() { return follower_dummy_->IsCanceled(); }, kTimeout));
  test_node_->CancelPathPlanAction();
}


/// If the goal is on a wall of the static map, it becomes ABORTED(GOAL_IS_ON_STATIC_OBSTACLE)
TEST_F(BasePathPlannerNodeTest, GoalIsOnStaticObstacle) {
  // Place the goal position on a wall of the static map
  goal_pose_.pose.position = static_map_wall_point_;
  // Execute route planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // Ends with ABORTED(GOAL_IS_ON_STATIC_OBSTACLE)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::GOAL_IS_ON_STATIC_OBSTACLE);
    }, kTimeout));
}


/// If the goal is outside the static map range, it becomes ABORTED(GOAL_IS_ON_STATIC_OBSTACLE)
TEST_F(BasePathPlannerNodeTest, GoalIsOutOfMap) {
  // Place the goal position outside the range of the static map
  goal_pose_.pose = CreatePose(-1.0, -1.0, 0.0);
  // Execute route planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // Ends with ABORTED(GOAL_IS_ON_STATIC_OBSTACLE)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::GOAL_IS_ON_STATIC_OBSTACLE);
    }, kTimeout));
}


/// If the robot position is outside the static map range, it becomes ABORTED(ROBOT_IS_OUT_OF_MAP)
TEST_F(BasePathPlannerNodeTest, RobotIsOutOfMap) {
  // Place the robot position outside the range of the static map
  global_pose_.pose = CreatePose(-1.0, -1.0, 0.0);
  cyclic_sender_->StartSendGlobalPose(global_pose_);
  // Execute route planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // Ends with ABORTED(ROBOT_IS_OUT_OF_MAP)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::ROBOT_IS_OUT_OF_MAP);
    }, kTimeout));
}

/// If the Follower becomes ABORTED, it becomes ABORTED(FOLLOWER_ABORTED)
TEST_F(BasePathPlannerNodeTest, FollowerAborted) {
  // Set follower dummy to become ABORTED after 1 second
  follower_dummy_->SetActionCompleteCondition(1.0, rclcpp_action::ResultCode::ABORTED);
  // Execute route planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // Becomes ABORTED(FOLLOWER_ABORTED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::FOLLOWER_ABORTED);
      }, kTimeout));
}

/// If the dynamic map times out, it becomes ABORTED(DYNAMIC_MAP_IS_NOT_UPDATED)
TEST_F(BasePathPlannerNodeTest, DynamicMapIsNotUpdated) {
  // Stop publishing the dynamic map
  cyclic_sender_->StopSendDynamicMap();
  // Execute route planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // Becomes ABORTED(DYNAMIC_MAP_IS_NOT_UPDATED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::DYNAMIC_MAP_IS_NOT_UPDATED);
    }, kTimeout));
}


/// If the self-position times out, it becomes ABORTED(ROBOT_POSE_IS_NOT_UPDATED)
TEST_F(BasePathPlannerNodeTest, RobotPoseIsNotUpdated) {
  // Stop publishing self-position
  cyclic_sender_->StopSendGlobalPose();
  // Execute route planning action
  test_node_->SendPathPlanAction(goal_pose_);
  // Becomes ABORTED(ROBOT_POSE_IS_NOT_UPDATED)
  EXPECT_TRUE(test_node_->WaitUntil([&]() {
      return test_node_->IsMatchActionResult(rclcpp_action::ResultCode::ABORTED,
      PathPlanAction::Result::ROBOT_POSE_IS_NOT_UPDATED);
    }, kTimeout));
}
}  // namespace tmc_base_path_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);
  // Generate base_path_planner node
  auto base_path_planner_node = std::make_shared<tmc_base_path_planner::BasePathPlannerNode>(option);
  // Read parameters from yaml
  const std::string yaml_directory =
      ament_index_cpp::get_package_share_directory("tmc_base_path_planner") + "/test/parameter/";
  LoadParameterFromYaml(base_path_planner_node, yaml_directory, "base_path_planner_node-test.yaml");
  base_path_planner_node->Init();
  // Create a thread to spin
  auto base_path_planner_node_thread = std::make_shared<std::thread>([&]() {
        rclcpp::spin(base_path_planner_node);
      });
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  base_path_planner_node_thread->join();
  base_path_planner_node.reset();

  return result;
}
