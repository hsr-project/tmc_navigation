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
/// @file base_path_follower_node-test.cpp
/// @brief Test for omnidirectional vehicle path-following node
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <angles/angles.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>

#include <tmc_navigation_msgs/action/path_follower.hpp>
// TODO(syuuhei_shiro): tmc_rostest_utilsをROS2化する
// #include <tmc_rostest_utils/util_function.hpp>
#include <tmc_base_path_follower/base_path_follower_node.hpp>
#include <tmc_base_path_follower/parameter_creator.hpp>
#include "robot_dummy_node.hpp"
#include "test_utils.hpp"

namespace {
// General timeout duration [s] Prevents infinite loops for processes that do not require timeout monitoring in tests
constexpr double kTimeOut = 30.0;
// Timeout duration for path-following [s]
constexpr double kPathFollowTimeOut = 100.0;
// Stop timeout duration [s] Countermeasure for topic overtaking between action result and final velocity
constexpr double kStopTimeOut = 0.5;
// Test drive cycle [s]
constexpr double kCycleTime = 0.01;
// Maximum tracking error monitored during movement [m]
constexpr double kMaxLinearErrorThreshold = 0.1;
// Position error allowed during stop [m]
constexpr double kGoalLinearErrorThreshold = 0.05;
// Angle error allowed during stop [rad]
constexpr double kRotationalErrorThreshold = 5.0 * M_PI / 180.0;
// Point interval of the input path [m]
constexpr double kPathInterval = 0.05;
// Path-following action name
constexpr const char* const kActionName = "path_follow_action";
// Maximum progress error monitored during movement
constexpr double kMaxProgressErrorThreshold = 0.1;
// Maximum allowable change in progress rate
constexpr double kMaxProgressChangeThreshold =  0.05;
// Distance [m] to determine that the robot has started moving after the action starts
constexpr double kMoveDetectionDistance = 0.5;
// Test curve path file
constexpr const char* const kCurvedPathFileName = "curve_path.yaml";
// Test two-lap path file
constexpr const char* const kTwoLapPathFileName = "two_lap_path.yaml";
// Time error allowed in self-position timeout test [s]
constexpr double kGlobalPoseTimeoutTolerance = 0.1;
}  // anonymous namespace

namespace tmc_base_path_follower {
using std::placeholders::_1;
using std::placeholders::_2;

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(const double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

/// Load path data from file
bool ReadPath(const std::string& path_file_name, nav_msgs::msg::Path& path) {
  std::string file_path = ament_index_cpp::get_package_share_directory("tmc_base_path_follower") +
      "/test/pathdata/" + path_file_name;
  std::ifstream ifs(file_path.c_str());
  if (ifs.fail()) {
    RCLCPP_ERROR(rclcpp::get_logger("base_path_follower_node-test"),
        "Failed to open path file. %s", file_path.c_str());
    return false;
  }
  // Read and store path points from file
  path.poses.clear();
  YAML::Node doc;
  YAML::Node x;
  YAML::Node y;
  YAML::Node t;
  try {
    doc = YAML::Load(ifs);
    x = doc["path"]["x"];
    y = doc["path"]["y"];
    t = doc["path"]["t"];
  } catch(const std::exception& e) {
     RCLCPP_ERROR(rclcpp::get_logger("base_path_follower_node-test"), "Failed to load yaml. %s", file_path.c_str());
    return false;
  }

  if (!x || !y || !t || x.size() != y.size() || x.size() != t.size()) {
     RCLCPP_ERROR(rclcpp::get_logger("base_path_follower_node-test"), "Invalid path data. %s", file_path.c_str());
    return false;
  }
  for (uint i = 0; i < x.size(); ++i) {
    geometry_msgs::msg::PoseStamped path_point;
    path_point.pose.position.x = x[i].as<double>();
    path_point.pose.position.y = y[i].as<double>();
    path_point.pose.orientation = createQuaternionMsgFromYaw(t[i].as<double>());
    path.poses.push_back(path_point);
  }

  path.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  return true;
}

/// Test node
class TestNode : public rclcpp::Node {
 public:
  using PathFollowActionClient = tmc_navigation_msgs::action::PathFollower;
  using PathFollowGoalHandle = rclcpp_action::ClientGoalHandle<PathFollowActionClient>;

  /// Constructor
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options), rate_(1.0 / kCycleTime), max_linear_error_(0.0),
      progress_(0.0), max_progress_error_(0.0), max_progress_change_(0.0) {}

  void Init(std::shared_ptr<BasePathFollowerNode> base_path_follower_node) {
    GetOptionalParam(shared_from_this(), "max_linear_error_threshold", max_linear_error_threshold_,
        kMaxLinearErrorThreshold);
    GetOptionalParam(shared_from_this(), "goal_linear_error_threshold", goal_linear_error_threshold_,
        kGoalLinearErrorThreshold);

    nodes_.push_back(shared_from_this());
    nodes_.push_back(base_path_follower_node);
    start_publish_pose_service_ = this->create_client<std_srvs::srv::Empty>("/start_publish_pose");
    stop_publish_pose_service_ = this->create_client<std_srvs::srv::Empty>("/stop_publish_pose");
    action_client_ = rclcpp_action::create_client<PathFollowActionClient>(this, kActionName);

    // Initial value of 0 may pass before subscription, so set to a value not used in tests
    global_pose_.pose.position.x = std::numeric_limits<double>::max();
    global_pose_.pose.position.y = std::numeric_limits<double>::max();
    velocity_.linear.x = std::numeric_limits<double>::max();
    velocity_.linear.y = std::numeric_limits<double>::max();
    velocity_.angular.z = std::numeric_limits<double>::max();

    // Test subscriber and publisher settings
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/base_path_follower/path", 1);
    pub_initial_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/initial_pose", 1);

    sub_global_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/global_pose", 1,
        std::bind(&TestNode::PoseCallback, this, _1));
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>("/base_velocity", 1,
        std::bind(&TestNode::VelocityCallback, this, _1));
    // Action state monitoring is required for testing when path is received
    // Only the action_client sending the action can check the action state, no public interface for external confirmation
    // Considered creating a function to determine state only for testing or using a hidden topic, decided to use a hidden topic
    sub_action_goal_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/path_follow_action/_action/status", 1, std::bind(&TestNode::ActionGoalStatusCallback, this, _1));

    // Start dummy self-position publishing
    StartPublishDummyPose();
    // Check if connected to target node
    ASSERT_TRUE(WaitForConnectionEstablished());
  }

  // Start dummy self-position publishing
  void StartPublishDummyPose() {
    auto req_start = std::make_shared<std_srvs::srv::Empty::Request>();
    start_publish_pose_service_->async_send_request(req_start);
  }

  // Stop dummy self-position publishing
  void StopPublishDummyPose() {
    auto req_stop = std::make_shared<std_srvs::srv::Empty::Request>();
    stop_publish_pose_service_->async_send_request(req_stop);
  }


  /// Wait until connection with target node is established
  bool WaitForConnectionEstablished() {
    return WaitUntil(nodes_,
        [&]() {
          return (pub_initial_pose_->get_subscription_count() > 0 &&
                  pub_path_->get_subscription_count() > 0 &&
                  sub_global_pose_->get_publisher_count() > 0 &&
                  sub_velocity_->get_publisher_count() > 0 &&
                  action_client_->action_server_is_ready());
        },
        kTimeOut);
  }

  /// Place robot dummy at initial position
  bool InitiateRobotPose(const geometry_msgs::msg::PoseStamped& initial_pose) {
    pub_initial_pose_->publish(initial_pose);
    return WaitUntil(nodes_,
        [&]() { return (CalcDistance(global_pose_, initial_pose) < kEpsilon); },
        kTimeOut);
  }

  /// Start path-following via action
  void SendPathFollowAction(const nav_msgs::msg::Path& path) {
    progress_ = 0.0;
    auto send_goal_options = rclcpp_action::Client<PathFollowActionClient>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&TestNode::GoalResponseCallback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&TestNode::FeedbackCallback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&TestNode::ResultCallback, this, _1);

    auto follower_goal = PathFollowActionClient::Goal();
    follower_goal.path = path;
    action_client_->async_send_goal(follower_goal, send_goal_options);
    ResetResultStatus();
  }

  // Cancel path-following via action
  void CancelFollowAction() {
    action_client_->async_cancel_all_goals();
  }

  /// Wait for path-following initiated by action to complete
  // Response drop occurs when ActionServer and ActionClient have the same cycle
  // Change ActionClient cycle to 200.0Hz to avoid test failure
  // TODO(kazuki_shibamiya) : actionの作りを見直し、ActionClientの周期を100.0Hzに戻す
  bool WaitForActionResult() {
    return WaitUntil(nodes_,
        [&]() {
          return (result_status_ != std::nullopt);
        },
        kPathFollowTimeOut,
        200.0);
  }

  void SpinOnce() {
    for (auto node : nodes_) {
      rclcpp::spin_some(node);
    }
  }

  /// Perform path-following via action and wait for completion
  bool SendPathFollowActionAndWaitComplete(const nav_msgs::msg::Path& path) {
    // Start action
    SendPathFollowAction(path);
    // Wait for action to complete
    return WaitFollowActionComplete(path);
  }

  // Wait for action to complete
  bool WaitFollowActionComplete(const nav_msgs::msg::Path& path) {
    const rclcpp::Time limit_time = this->get_clock()->now() +
        rclcpp::Duration::from_seconds(kPathFollowTimeOut);
    max_linear_error_ = 0.0;
    max_progress_error_ = 0.0;
    while (result_status_ == std::nullopt) {
      // Calculate maximum error between nearest point on path and self-position
      double distance_to_nearest_point = DistancePoseToPath(global_pose_, path);
      if (max_linear_error_ < distance_to_nearest_point) {
        max_linear_error_ = distance_to_nearest_point;
      }
      // Estimate progress from nearest point and calculate maximum error with feedback
      const uint32_t nearest_index = NearestIndex(global_pose_, path);
      const double estimate_progress = static_cast<double>(nearest_index) / path.poses.size();
      const double progress_error = fabs(estimate_progress - progress_);
      if (max_progress_error_ < progress_error) {
        max_progress_error_ = progress_error;
      }
      // Path-following fails if goal is not reached within a certain time
      if (this->get_clock()->now() > limit_time) {
        RCLCPP_WARN(rclcpp::get_logger("base_path_follower_node-test"), "Path following timeout.");
        return false;
      }
      SpinOnce();
    }
    return true;
  }

  /// Wait for output velocity to become 0
  bool WaitToStop() {
    return WaitUntil(nodes_,
        [&]() {
          return (velocity_.linear.x < std::numeric_limits<double>::epsilon() &&
              velocity_.linear.y < std::numeric_limits<double>::epsilon() &&
              velocity_.angular.z < std::numeric_limits<double>::epsilon());
        },
        kStopTimeOut);
  }

  /// Perform path-following via topic and wait until stop. Does not determine if goal was reached correctly
  bool PublishPathAndWaitComplete(const nav_msgs::msg::Path& path,
      const int8_t expected_result_status) {
    // Publish path
    action_goal_status_ = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
    pub_path_->publish(path);
    // Wait until path-following reaches the goal
    const rclcpp::Time limit_time = this->get_clock()->now() +
        rclcpp::Duration::from_seconds(kPathFollowTimeOut);
    max_linear_error_ = 0.0;
    while (action_goal_status_ != expected_result_status) {
      // Calculate maximum error between nearest point on path and self-position
      const double distance_to_nearest_point = DistancePoseToPath(global_pose_, path);
      if (max_linear_error_ < distance_to_nearest_point) {
        max_linear_error_ = distance_to_nearest_point;
      }
      // Path-following fails if goal is not reached within a certain time
      if (this->get_clock()->now() > limit_time) {
        RCLCPP_WARN(rclcpp::get_logger("base_path_follower_node-test"), "Path following timeout.");
        return false;
      }
      rate_.sleep();
      SpinOnce();
    }

    return true;
  }

  /// Wait for the robot to move a specified distance
  bool WaitToMove() {
    const geometry_msgs::msg::PoseStamped initial_pose = global_pose_;

    return WaitUntil(nodes_,
        [&]() {
          return (CalcDistance(global_pose_, initial_pose) > kMoveDetectionDistance);
        },
        kPathFollowTimeOut);
  }

  // Reset path-following action client result
  void ResetResultStatus() {
    result_status_ = std::nullopt;
  }

  // Publish tracking path
  void PublishPath(const nav_msgs::msg::Path& path) {
    pub_path_->publish(path);
  }

  /// Accessor
  // Self-position
  geometry_msgs::msg::PoseStamped global_pose() const { return global_pose_; }
  // Path-following action client result
  std::optional<rclcpp_action::ResultCode> result_status() const { return result_status_; }
  // Maximum path-following error
  double max_linear_error() const { return max_linear_error_; }
  // Maximum progress error
  double max_progress_error() const { return max_progress_error_; }
  // Maximum progress change
  double max_progress_change() const { return max_progress_change_; }
  // Maximum tracking error monitored during movement [m]
  double max_linear_error_threshold() const { return max_linear_error_threshold_; }
  // Position error allowed during stop [m]
  double goal_linear_error_threshold() const { return goal_linear_error_threshold_; }

 private:
  /// Self-position callback
  void PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    global_pose_ = *pose;
  }

  /// Vehicle velocity command callback
  void VelocityCallback(const geometry_msgs::msg::Twist::SharedPtr velocity) {
    velocity_ = *velocity;
  }

  // Action goal response callback
  void GoalResponseCallback(const PathFollowGoalHandle::SharedPtr& future) {}
  /// Action completion callback
  void ResultCallback(const PathFollowGoalHandle::WrappedResult& result) {
    result_status_ = result.code;
  }
  /// Action progress callback
  void FeedbackCallback(PathFollowGoalHandle::SharedPtr,
      const std::shared_ptr<const PathFollowActionClient::Feedback> feedback) {
    const double progress_change = fabs(progress_ - feedback->progress);
    if (max_progress_change_ < progress_change) {
      // Record maximum progress change
      max_progress_change_ = progress_change;
    }

    progress_ = feedback->progress;
  }
  // Action state topic callback
  void ActionGoalStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
    if (!msg->status_list.empty()) {
      action_goal_status_ = msg->status_list.back().status;
    }
  }

  // Cycle wait
  rclcpp::Rate rate_;
  // Tracking path publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  // Test start position publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_initial_pose_;
  // Self-position subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_global_pose_;
  // Vehicle velocity command subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_velocity_;
  // Path-following action client
  rclcpp_action::Client<PathFollowActionClient>::SharedPtr action_client_;
  // Path-following action client result
  std::optional<rclcpp_action::ResultCode> result_status_;
  // Path-following action topic result
  int8_t action_goal_status_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr sub_action_goal_;


  std::vector<std::shared_ptr<rclcpp::Node>> nodes_;

  // Self-position
  geometry_msgs::msg::PoseStamped global_pose_;
  // Vehicle velocity command value
  geometry_msgs::msg::Twist velocity_;
  // Action progress rate
  double progress_;
  // Maximum path-following error
  double max_linear_error_;
  // Maximum progress error
  double max_progress_error_;
  // Maximum progress change
  double max_progress_change_;
  // Maximum tracking error monitored during movement [m]
  double max_linear_error_threshold_;
  // Position error allowed during stop [m]
  double goal_linear_error_threshold_;

  // Start/stop service for dummy self-position
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_publish_pose_service_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_publish_pose_service_;
};

/// BasePathFollower node test fixture
class BasePathFollowerNodeTest : public testing::Test {
 public:
  BasePathFollowerNodeTest() {}

 protected:
  /// Test initial setup
  virtual void SetUp() {
    std::string parameter_file = std::string(getenv("PARAMETER_FILE"));
    parameter_file.erase(0, 1);
    // Common coordinates for tests without specific start/goal positions
    start_pose_.pose.position.x = 0.0;
    start_pose_.pose.position.y = 0.0;
    start_pose_.pose.orientation = createQuaternionMsgFromYaw(0.0);
    goal_pose_.pose.position.x = 1.0;
    goal_pose_.pose.position.y = 1.0;
    goal_pose_.pose.orientation = createQuaternionMsgFromYaw(M_PI / 2.0);

    const std::string yaml_directory =
        ament_index_cpp::get_package_share_directory("tmc_base_path_follower") + "/test/parameter/";
    rclcpp::NodeOptions option;
    option.allow_undeclared_parameters();
    option.automatically_declare_parameters_from_overrides(true);
    base_path_follower_node_ = std::make_shared<BasePathFollowerNode>(option);
    LoadParameterFromYaml(base_path_follower_node_, yaml_directory, parameter_file);
    base_path_follower_node_->Init();
    robot_dummy_node_ = std::make_shared<RobotDummyNode>(option);
    robot_dummy_node_->Init();
    robot_dummy_thread_ = std::make_shared<std::thread>([&]() { robot_dummy_node_->Run(); });
    test_node_ = std::make_shared<TestNode>(option);
    LoadParameterFromYaml(test_node_, yaml_directory, parameter_file);
    test_node_->Init(base_path_follower_node_);
  }
  /// Test cleanup
  virtual void TearDown() {
    robot_dummy_node_->Kill();
    robot_dummy_thread_->join();
  }

  std::shared_ptr<BasePathFollowerNode> base_path_follower_node_;
  std::shared_ptr<TestNode> test_node_;
  std::shared_ptr<RobotDummyNode> robot_dummy_node_;
  std::shared_ptr<std::thread> robot_dummy_thread_;

  // Start/goal positions
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
};

/// Normal case confirmation for path-following via action
/// Check if action ends normally and progress report is accurate
TEST_F(BasePathFollowerNodeTest, ActionNormalOperation) {
  // setup
  // Create straight path
  const nav_msgs::msg::Path linear_path = CreateLinearPath(start_pose_, goal_pose_, kPathInterval);
  // Place robot at start position
  ASSERT_TRUE(test_node_->InitiateRobotPose(start_pose_));
  // exercise
  // Check if path-following can be completed for a straight path
  ASSERT_TRUE(test_node_->SendPathFollowActionAndWaitComplete(linear_path));

  // verify
  // Check if action stops with SUCCEEDED
  ASSERT_NE(test_node_->result_status(), std::nullopt);
  EXPECT_EQ(test_node_->result_status().value(), rclcpp_action::ResultCode::SUCCEEDED);

  // Check if maximum tracking error is below specified value
  EXPECT_LT(test_node_->max_linear_error(), test_node_->max_linear_error_threshold());
  // Check if maximum progress rate error during movement is below specified value
  EXPECT_LT(test_node_->max_progress_error(), kMaxProgressErrorThreshold);
  // Check if final goal position and angle error are below specified value
  const double error_linear = CalcDistance(goal_pose_, test_node_->global_pose());
  const double error_t = angles::shortest_angular_distance(tf2::getYaw(goal_pose_.pose.orientation),
                                                           tf2::getYaw(test_node_->global_pose().pose.orientation));
  EXPECT_LT(error_linear, test_node_->goal_linear_error_threshold());
  EXPECT_LT(fabs(error_t), kRotationalErrorThreshold);
}

/// Short-distance straight path-following test
/// Generate random short-distance straight path and follow it
/// Check if goal can be reached correctly with error below a certain value
TEST_F(BasePathFollowerNodeTest, RandomLinearPath) {
  // setup
  // Randomly set goal position
  std::mt19937 rng(static_cast<unsigned int>(time(0)));
  // Translation random value
  std::uniform_real_distribution<> linear_dist(0.0, 2.0);
  auto linear_rand = [&]() { return linear_dist(rng); };
  // Rotation random value
  std::uniform_real_distribution<> angular_dist(-M_PI, M_PI);
  auto angular_rand = [&]() { return angular_dist(rng); };
  // Set goal
  goal_pose_.pose.position.x = start_pose_.pose.position.x + linear_rand();
  goal_pose_.pose.position.y = start_pose_.pose.position.y + linear_rand();
  goal_pose_.pose.orientation = createQuaternionMsgFromYaw(angular_rand());
  // Create straight path
  const nav_msgs::msg::Path linear_path = CreateLinearPath(start_pose_, goal_pose_, kPathInterval);
  // Log randomly generated goal for test reproducibility
  RCLCPP_INFO(rclcpp::get_logger("base_path_follower_node-test"), "RamdomLinearPath goal: (%f, %f, %f)",
      goal_pose_.pose.position.x, goal_pose_.pose.position.y, tf2::getYaw(goal_pose_.pose.orientation));
  // Place robot at start position
  ASSERT_TRUE(test_node_->InitiateRobotPose(start_pose_));

  // exercise
  // Check if path-following can be completed for a straight path
  ASSERT_TRUE(test_node_->SendPathFollowActionAndWaitComplete(linear_path));

  // verify
  // Check if action stops with SUCCEEDED
  ASSERT_NE(test_node_->result_status(), std::nullopt);
  EXPECT_EQ(test_node_->result_status().value(), rclcpp_action::ResultCode::SUCCEEDED);

  // Check if maximum tracking error is below specified value
  EXPECT_LT(test_node_->max_linear_error(), test_node_->max_linear_error_threshold());
  // Check if final goal position and angle error are below specified value
  const double error_linear = CalcDistance(goal_pose_, test_node_->global_pose());
  const double error_t = angles::shortest_angular_distance(
      tf2::getYaw(goal_pose_.pose.orientation), tf2::getYaw(test_node_->global_pose().pose.orientation));
  EXPECT_LT(error_linear, test_node_->goal_linear_error_threshold());
  EXPECT_LT(fabs(error_t), kRotationalErrorThreshold);
}

/// Curved path-following test
/// Load path including curves from file and follow it
/// Check if goal can be reached correctly with error below a certain value
TEST_F(BasePathFollowerNodeTest, FollowCurvePath) {
  // setup
  // Load path file including curves
  nav_msgs::msg::Path curve_path;
  ASSERT_TRUE(ReadPath(kCurvedPathFileName, curve_path));
  // Place robot at start position
  ASSERT_TRUE(test_node_->InitiateRobotPose(curve_path.poses.front()));

  // exercise
  // Check if path-following can be completed for the specified path
  ASSERT_TRUE(test_node_->SendPathFollowActionAndWaitComplete(curve_path));

  // verify
  // Check if action stops with SUCCEEDED
  ASSERT_NE(test_node_->result_status(), std::nullopt);
  EXPECT_EQ(test_node_->result_status().value(), rclcpp_action::ResultCode::SUCCEEDED);
  // Check if maximum tracking error is below specified value
  EXPECT_LT(test_node_->max_linear_error(), test_node_->max_linear_error_threshold());
  // Check if final goal position and angle error are below specified value
  const double error_linear = CalcDistance(curve_path.poses.back(), test_node_->global_pose());
  const double error_t = angles::shortest_angular_distance(
      tf2::getYaw(curve_path.poses.back().pose.orientation),
      tf2::getYaw(test_node_->global_pose().pose.orientation));
  EXPECT_LT(error_linear, test_node_->goal_linear_error_threshold());
  EXPECT_LT(fabs(error_t), kRotationalErrorThreshold);
}

/// Path-following test for running the same location twice
/// Load path from file and follow it
/// Check if path is followed in order
/// Check if goal can be reached correctly
TEST_F(BasePathFollowerNodeTest, FollowTwoLapPath) {
  // setup
  // Load path file for running the same location twice
  nav_msgs::msg::Path two_lap_path;
  ASSERT_TRUE(ReadPath(kTwoLapPathFileName, two_lap_path));
  // Place robot at start position
  ASSERT_TRUE(test_node_->InitiateRobotPose(two_lap_path.poses.front()));

  // exercise
  // Check if path-following can be completed for the specified path
  ASSERT_TRUE(test_node_->SendPathFollowActionAndWaitComplete(two_lap_path));

  // verify
  /// The path used in this test involves two laps of a small circle with a radius of 25cm
  /// Therefore, the tracking error condition is stricter than the range we want to verify
  /// Maximum tracking error check will not be performed
  // Check if action stops with SUCCEEDED
  ASSERT_NE(test_node_->result_status(), std::nullopt);
  EXPECT_EQ(test_node_->result_status().value(), rclcpp_action::ResultCode::SUCCEEDED);
  // Check if path is followed in order. Confirm progress change is within threshold
  EXPECT_LT(test_node_->max_progress_change(), kMaxProgressChangeThreshold);
  // Check if final goal position and angle error are below specified value
  const double error_linear = CalcDistance(two_lap_path.poses.back(), test_node_->global_pose());
  const double error_t = angles::shortest_angular_distance(
      tf2::getYaw(two_lap_path.poses.back().pose.orientation),
      tf2::getYaw(test_node_->global_pose().pose.orientation));
  EXPECT_LT(error_linear, test_node_->goal_linear_error_threshold());
  EXPECT_LT(fabs(error_t), kRotationalErrorThreshold);
}

/// Check if action stops when canceled
TEST_F(BasePathFollowerNodeTest, CancelAction) {
  // setup
  // Create straight path
  const nav_msgs::msg::Path linear_path = CreateLinearPath(start_pose_, goal_pose_, kPathInterval);
  // Place robot at start position
  ASSERT_TRUE(test_node_->InitiateRobotPose(start_pose_));

  // exercise
  // Issue action and wait until movement starts
  test_node_->SendPathFollowAction(linear_path);
  EXPECT_TRUE(test_node_->WaitToMove());
  // Issue cancel
  test_node_->CancelFollowAction();
  // Wait for action to complete
  ASSERT_TRUE(test_node_->WaitForActionResult());
  // verify
  // Check if action stops with CANCELED
  ASSERT_NE(test_node_->result_status(), std::nullopt);
  EXPECT_EQ(test_node_->result_status().value(), rclcpp_action::ResultCode::CANCELED);
  // Check if current velocity is 0 and stopped
  EXPECT_TRUE(test_node_->WaitToStop());
}

/// Input another action during action execution
/// Check if goal can be reached correctly for subsequent action
/// Check if goal determination corresponding to the later input path is returned
TEST_F(BasePathFollowerNodeTest, UpdateActionByAction) {
  // setup
  // Create straight path
  const nav_msgs::msg::Path linear_path = CreateLinearPath(start_pose_, goal_pose_, kPathInterval);
  // Place robot at start position
  ASSERT_TRUE(test_node_->InitiateRobotPose(start_pose_));

  // exercise
  // Issue action and wait until robot starts moving
  test_node_->SendPathFollowAction(linear_path);
  EXPECT_TRUE(test_node_->WaitToMove());
  // Generate path to another goal from current position
  geometry_msgs::msg::PoseStamped second_goal_pose;
  second_goal_pose.pose.position.x = 2.0;
  second_goal_pose.pose.position.y = -1.0;
  second_goal_pose.pose.orientation = createQuaternionMsgFromYaw(0.0);
  const nav_msgs::msg::Path second_linear_path = CreateLinearPath(
      test_node_->global_pose(), second_goal_pose, kPathInterval);
  // Issue new path
  test_node_->SendPathFollowAction(second_linear_path);
  // Confirm that the earlier action becomes ABORTED
  ASSERT_TRUE(test_node_->WaitForActionResult());
  ASSERT_NE(test_node_->result_status(), std::nullopt);
  EXPECT_EQ(test_node_->result_status().value(), rclcpp_action::ResultCode::ABORTED);
  test_node_->ResetResultStatus();
  // Check if path-following can be completed for the new path
  test_node_->WaitFollowActionComplete(second_linear_path);

  // verify
  // Check if subsequent action ends with SUCCEEDED
  ASSERT_NE(test_node_->result_status(), std::nullopt);
  EXPECT_EQ(test_node_->result_status().value(), rclcpp_action::ResultCode::SUCCEEDED);
  // Check if the goal corresponding to the subsequent action is reached
  const double error_linear = CalcDistance(second_goal_pose, test_node_->global_pose());
  const double error_t = angles::shortest_angular_distance(
      tf2::getYaw(second_goal_pose.pose.orientation), tf2::getYaw(test_node_->global_pose().pose.orientation));
  EXPECT_LT(error_linear, test_node_->goal_linear_error_threshold());
  EXPECT_LT(fabs(error_t), kRotationalErrorThreshold);
}

/// Abnormal case confirmation for input path
/// Check if action returns ABORTED when a path with less than 2 points is input
TEST_F(BasePathFollowerNodeTest, InvalidGoal) {
  // setup
  // Create single-point path
  nav_msgs::msg::Path one_point_path;
  one_point_path.header.stamp = test_node_->get_clock()->now();
  one_point_path.poses.push_back(start_pose_);
  // Place robot at start position
  ASSERT_TRUE(test_node_->InitiateRobotPose(start_pose_));

  // exercise
  // Input single-point path and wait for action to complete
  ASSERT_TRUE(test_node_->SendPathFollowActionAndWaitComplete(one_point_path));

  // verify
  // Check if action stops with ABORTED
  ASSERT_NE(test_node_->result_status(), std::nullopt);
  EXPECT_EQ(test_node_->result_status().value(), rclcpp_action::ResultCode::ABORTED);
}

/// Interrupt with another invalid goal during action execution
/// Check if robot stops
TEST_F(BasePathFollowerNodeTest, PreemptedByInvalidGoal) {
  // setup
  // Create straight path
  const nav_msgs::msg::Path linear_path = CreateLinearPath(start_pose_, goal_pose_, kPathInterval);
  // Place robot at start position
  ASSERT_TRUE(test_node_->InitiateRobotPose(start_pose_));

  // exercise
  // Issue action and wait until robot starts moving
  test_node_->SendPathFollowAction(linear_path);
  EXPECT_TRUE(test_node_->WaitToMove());
  // Create single-point path at current position
  nav_msgs::msg::Path one_point_path;
  one_point_path.header.stamp = test_node_->get_clock()->now();
  one_point_path.poses.push_back(test_node_->global_pose());
  // Reissue action with new path
  test_node_->SendPathFollowAction(one_point_path);
  // Confirm that the earlier action becomes ABORTED
  ASSERT_TRUE(test_node_->WaitForActionResult());
  ASSERT_NE(test_node_->result_status(), std::nullopt);
  EXPECT_EQ(test_node_->result_status().value(), rclcpp_action::ResultCode::ABORTED);
  test_node_->ResetResultStatus();

  // verify
  // Check if subsequent action ends with ABORTED
  ASSERT_TRUE(test_node_->WaitForActionResult());
  ASSERT_NE(test_node_->result_status(), std::nullopt);
  EXPECT_EQ(test_node_->result_status().value(), rclcpp_action::ResultCode::ABORTED);
  // Check if current velocity is 0 and stopped
  EXPECT_TRUE(test_node_->WaitToStop());
}

/// Normal case confirmation for path-following via topic
/// Check if robot stops at goal coordinates
TEST_F(BasePathFollowerNodeTest, TopicNormalOperation) {
  // setup
  // Create straight path
  const nav_msgs::msg::Path linear_path = CreateLinearPath(start_pose_, goal_pose_, kPathInterval);
  // Place robot at start position
  ASSERT_TRUE(test_node_->InitiateRobotPose(start_pose_));

  // exercise
  // Check if path-following can be completed for a straight path
  ASSERT_TRUE(test_node_->PublishPathAndWaitComplete(linear_path, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  // verify
  // Check if maximum tracking error is below specified value
  EXPECT_LT(test_node_->max_linear_error(), test_node_->max_linear_error_threshold());
  // Check if maximum progress rate error during movement is below specified value
  EXPECT_LT(test_node_->max_progress_error(), kMaxProgressErrorThreshold);
  // Check if final goal position and angle error are below specified value
  const double error_linear = CalcDistance(goal_pose_, test_node_->global_pose());
  const double error_t = angles::shortest_angular_distance(tf2::getYaw(goal_pose_.pose.orientation),
                                                           tf2::getYaw(test_node_->global_pose().pose.orientation));
  EXPECT_LT(error_linear, test_node_->goal_linear_error_threshold());
  EXPECT_LT(fabs(error_t), kRotationalErrorThreshold);
}

/// Overwriting path-following via topic
/// Check if goal determination corresponding to the later input path is returned
TEST_F(BasePathFollowerNodeTest, UpdateTopicByTopic) {
  // setup
  // Create straight path
  const nav_msgs::msg::Path linear_path = CreateLinearPath(start_pose_, goal_pose_, kPathInterval);
  // Place robot at start position
  ASSERT_TRUE(test_node_->InitiateRobotPose(start_pose_));

  // exercise
  // Publish path and wait until robot starts moving
  test_node_->PublishPath(linear_path);
  EXPECT_TRUE(test_node_->WaitToMove());
  // Generate path to another goal from current position
  geometry_msgs::msg::PoseStamped second_goal_pose;
  second_goal_pose.pose.position.x = 2.0;
  second_goal_pose.pose.position.y = -1.0;
  second_goal_pose.pose.orientation = createQuaternionMsgFromYaw(0.0);
  const nav_msgs::msg::Path second_linear_path = CreateLinearPath(test_node_->global_pose(),
                                                                  second_goal_pose, kPathInterval);
  // Issue new path and check if path-following can be completed for it
  ASSERT_TRUE(test_node_->PublishPathAndWaitComplete(second_linear_path,
              action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  // Check if the new goal is reached
  const double error_linear = CalcDistance(second_goal_pose, test_node_->global_pose());
  const double error_t = angles::shortest_angular_distance(tf2::getYaw(second_goal_pose.pose.orientation),
                                                           tf2::getYaw(test_node_->global_pose().pose.orientation));
  EXPECT_LT(error_linear, test_node_->goal_linear_error_threshold());
  EXPECT_LT(fabs(error_t), kRotationalErrorThreshold);
}

/// Self-position timeout
/// Check if action becomes ABORTED and robot stops when self-position is lost during action execution
TEST_F(BasePathFollowerNodeTest, GlobalPoseTimeout) {
  // setup
  // Create straight path
  const nav_msgs::msg::Path linear_path = CreateLinearPath(start_pose_, goal_pose_, kPathInterval);
  // Place robot at start position
  ASSERT_TRUE(test_node_->InitiateRobotPose(start_pose_));

  // exercise
  // Issue action and wait until movement starts
  test_node_->SendPathFollowAction(linear_path);
  EXPECT_TRUE(test_node_->WaitToMove());
  const rclcpp::Time stop_global_pose_time = test_node_->get_clock()->now();
  // Stop dummy self-position publishing
  test_node_->StopPublishDummyPose();

  // Wait for action to complete
  ASSERT_TRUE(test_node_->WaitForActionResult());
  const double elapsed_time = (test_node_->get_clock()->now() - stop_global_pose_time).seconds();

  // verify
  // Check if action stops with ABORTED
  ASSERT_NE(test_node_->result_status(), std::nullopt);
  EXPECT_EQ(test_node_->result_status().value(), rclcpp_action::ResultCode::ABORTED);

  // Check if current velocity is 0 and stopped
  EXPECT_TRUE(test_node_->WaitToStop());
  // Confirm timeout occurs after specified time if a valid timeout parameter is set
  double global_pose_timeout;
  if (GetParam(base_path_follower_node_, "global_pose_timeout", global_pose_timeout) &&
      global_pose_timeout > std::numeric_limits<double>::epsilon()) {
    EXPECT_GT(elapsed_time, global_pose_timeout);
    EXPECT_LT(elapsed_time, global_pose_timeout + kGlobalPoseTimeoutTolerance);
  }
}

/// When the nearest point is the goal but the robot is a certain distance away from the goal
/// Considered as overrun, check if action becomes ABORTED and robot stops
TEST_F(BasePathFollowerNodeTest, OverRunning) {
  // setup
  // Create straight path
  const nav_msgs::msg::Path linear_path = CreateLinearPath(start_pose_, goal_pose_, kPathInterval);
  // Place robot at a position beyond the goal
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.position.x = goal_pose_.pose.position.x + 1.0;
  robot_pose.pose.position.y = goal_pose_.pose.position.y + 1.0;
  robot_pose.pose.orientation = createQuaternionMsgFromYaw(0.0);
  ASSERT_TRUE(test_node_->InitiateRobotPose(robot_pose));

  // exercise
  // Issue action and wait until movement starts
  test_node_->SendPathFollowAction(linear_path);
  // Wait for action to complete
  ASSERT_TRUE(test_node_->WaitForActionResult());

  // verify
  // Check if action stops with ABORTED
  ASSERT_NE(test_node_->result_status(), std::nullopt);
  EXPECT_EQ(test_node_->result_status().value(), rclcpp_action::ResultCode::ABORTED);

  // Check if current velocity is 0 and stopped
  EXPECT_TRUE(test_node_->WaitToStop());
}
}  // namespace tmc_base_path_follower


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
