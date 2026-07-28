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
/// @file viewpoint_controller-test.cpp
/// @brief Test of the viewpoint control node

#include <chrono>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <tmc_viewpoint_controller/viewpoint_controller_node.hpp>


namespace {
constexpr double kTimeout = 10.0;
constexpr double kNoResultTimeout = 1.0;
constexpr double kRate = 10.0;
constexpr double kThreshEqual = 0.0001;
constexpr const char* const kNeckYawName = "head_pan_joint";
constexpr const char* const kNeckPitchName = "head_tilt_joint";
constexpr double kMaxRotationOnceRad = 0.6;    // Maximum neck pan axis rotation per cycle [rad]

constexpr double kForwardX = 0.3;       // Forward distance on X-axis [m]
constexpr double kNearGoalX = 0.5;      // Near goal distance on X-axis (left diagonal path) [m]
constexpr double kNearGoalY = 0.5;      // Near goal distance on Y-axis (left diagonal path) [m]
constexpr double kCloseGoalX = 0.7;     // Close goal distance on X-axis (left diagonal path) [m]
constexpr double kCloseGoalY = 0.7;     // Close goal distance on Y-axis (left diagonal path) [m]
constexpr double kBackX = -2.0;         // Backward distance on X-axis [m]

constexpr double kTurnRightDeg = -5.0;  // Right turn angle [deg]
constexpr double kTurnLeftDeg = 5.0;    // Left turn angle [deg]

constexpr double kNeckRightDeg = -10.0;  // Right neck pan angle [deg]
constexpr double kNeckLeftDeg = 10.0;    // Left neck pan angle [deg]

inline double Deg2Rad(const double deg) { return deg * M_PI / 180.0; }

tf2::Quaternion CreateQuaternionFromYaw(const double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return q;
}

void CreateGlobalPoseOrigin(tf2::Transform& robot_pose) {
  robot_pose.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  robot_pose.setRotation(CreateQuaternionFromYaw(0.0));
}

void CreateGlobalPosePosition(tf2::Transform& robot_pose, const double x, const double y) {
  // xy-axis forward
  robot_pose.setOrigin(tf2::Vector3(x, y, 0.0));
  robot_pose.setRotation(CreateQuaternionFromYaw(0.0));
}

void CreateGlobalPoseTurn(tf2::Transform& robot_pose, const double rad) {
  // turn
  robot_pose.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  robot_pose.setRotation(CreateQuaternionFromYaw(rad));
}

void CreateNeckPose(sensor_msgs::msg::JointState& joint_states, const double rad) {
  joint_states.name.resize(2);
  joint_states.name[0] = kNeckYawName;
  joint_states.name[1] = kNeckPitchName;
  joint_states.position.resize(2);
  joint_states.position[0] = rad;
  joint_states.position[1] = 0.0;
}

// Generate straight path
void CreateStraightPath(nav_msgs::msg::Path& path) {
  path.poses.resize(5);
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[1].pose.position.x = 0.5;
  path.poses[1].pose.position.y = 0.0;
  path.poses[2].pose.position.x = 1.0;
  path.poses[2].pose.position.y = 0.0;
  path.poses[3].pose.position.x = 1.5;
  path.poses[3].pose.position.y = 0.0;
  path.poses[4].pose.position.x = 2.0;
  path.poses[4].pose.position.y = 0.0;
}

// Generate right curve path
void CreateRightCurvePath(nav_msgs::msg::Path& path) {
  path.poses.resize(5);
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[1].pose.position.x = 0.3;
  path.poses[1].pose.position.y = 0.0;
  path.poses[2].pose.position.x = 0.5;
  path.poses[2].pose.position.y = -0.05;
  path.poses[3].pose.position.x = 0.7;
  path.poses[3].pose.position.y = -0.15;
  path.poses[4].pose.position.x = 0.9;
  path.poses[4].pose.position.y = -0.3;
}

// Generate left curve path
void CreateLeftCurvePath(nav_msgs::msg::Path& path) {
  path.poses.resize(5);
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[1].pose.position.x = 0.3;
  path.poses[1].pose.position.y = 0.0;
  path.poses[2].pose.position.x = 0.5;
  path.poses[2].pose.position.y = 0.05;
  path.poses[3].pose.position.x = 0.7;
  path.poses[3].pose.position.y = 0.15;
  path.poses[4].pose.position.x = 0.9;
  path.poses[4].pose.position.y = 0.3;
}

// Generate right diagonal path
void CreateRightSlopePath(nav_msgs::msg::Path& path) {
  path.poses.resize(5);
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[1].pose.position.x = 0.3;
  path.poses[1].pose.position.y = -0.3;
  path.poses[2].pose.position.x = 0.5;
  path.poses[2].pose.position.y = -0.5;
  path.poses[3].pose.position.x = 0.7;
  path.poses[3].pose.position.y = -0.7;
  path.poses[4].pose.position.x = 0.9;
  path.poses[4].pose.position.y = -0.9;
}

// Generate left diagonal path
void CreateLeftSlopePath(nav_msgs::msg::Path& path) {
  path.poses.resize(5);
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[1].pose.position.x = 0.3;
  path.poses[1].pose.position.y = 0.3;
  path.poses[2].pose.position.x = 0.5;
  path.poses[2].pose.position.y = 0.5;
  path.poses[3].pose.position.x = 0.7;
  path.poses[3].pose.position.y = 0.7;
  path.poses[4].pose.position.x = 0.9;
  path.poses[4].pose.position.y = 0.9;
}

// Generate backward path
void CreateBackPath(nav_msgs::msg::Path& path) {
  path.poses.resize(5);
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[1].pose.position.x = -0.5;
  path.poses[1].pose.position.y = 0.0;
  path.poses[2].pose.position.x = -1.0;
  path.poses[2].pose.position.y = 0.0;
  path.poses[3].pose.position.x = -1.5;
  path.poses[3].pose.position.y = 0.0;
  path.poses[4].pose.position.x = -2.0;
  path.poses[4].pose.position.y = 0.0;
}

// Generate 0 path
void CreateZeroPath(nav_msgs::msg::Path& path) { path.poses.clear(); }

}  // anonymous namespace


namespace tmc_viewpoint_controller {
using std::placeholders::_1;

class TestNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options) {}

  /// Initialization
  void Init() {
    global_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("global_pose", 1);
    base_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("base_local_path", 1);
    target_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("target_path", 1);
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    viewpoint_result_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/command", 1,
        std::bind(&TestNode::ViewpointResultCallback, this, _1));
    start_service_client_ = this->create_client<std_srvs::srv::Empty>("/viewpoint_controller/start");
    stop_service_client_ = this->create_client<std_srvs::srv::Empty>("/viewpoint_controller/stop");
    set_viewpoint_mode_path_service_client_ =
        this->create_client<std_srvs::srv::Empty>("/viewpoint_controller/set_viewpoint_mode_path");
    set_viewpoint_mode_tracking_service_client_ =
        this->create_client<std_srvs::srv::Empty>("/viewpoint_controller/set_viewpoint_mode_tracking");

    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(shared_from_this()));
  }

  void ClearPath() {
    // Reset path
    nav_msgs::msg::Path path;
    CreateZeroPath(path);
    PublishBasePath(path);
    PublishTargetPath(path);
  }

  bool WaitForConnectionEstablished() {
    rclcpp::Rate rate(kRate);
    const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
    while (viewpoint_result_sub_->get_publisher_count() == 0 || base_path_pub_->get_subscription_count() == 0 ||
           target_path_pub_->get_subscription_count() == 0 || joint_states_pub_->get_subscription_count() == 0) {
      if (rclcpp::Clock(RCL_ROS_TIME).now() - start_time > rclcpp::Duration::from_seconds(kTimeout)) {
        RCLCPP_FATAL(this->get_logger(), "Can not link to target.");
        return false;
      }
      SpinOnce();
      rate.sleep();
    }
    return true;
  }

  bool WaitForResult(const double timeout) {
    is_sub_result_ = false;
    rclcpp::Rate rate(kRate);
    const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
    while (!is_sub_result_ &&
        rclcpp::Clock(RCL_ROS_TIME).now() - start_time < rclcpp::Duration::from_seconds(timeout)) {
      SpinOnce();
      rate.sleep();
    }
    return is_sub_result_;
  }

  void CallStartService() {
    CallEmptyService(start_service_client_);
  }
  void CallStopService() {
    CallEmptyService(stop_service_client_);
  }
  void CallSetViewpointModePathService() {
    CallEmptyService(set_viewpoint_mode_path_service_client_);
  }
  void CallSetViewpointTrackingTargetService() {
    CallEmptyService(set_viewpoint_mode_tracking_service_client_);
  }
  // Spin the test node
  void SpinOnce() {
    rclcpp::spin_some(shared_from_this());
  }

  void PublishJointStates(const sensor_msgs::msg::JointState& joint_states) {
    joint_states_pub_->publish(joint_states);
  }

  void PublishBasePath(const nav_msgs::msg::Path& base_path) {
    base_path_pub_->publish(base_path);
  }

  void PublishTargetPath(const nav_msgs::msg::Path& target_path) {
    target_path_pub_->publish(target_path);
  }

  void SendTransform(const tf2::Transform transform, const std::string& frame_id, const std::string& child_frame_id) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    transform_stamped.transform = tf2::toMsg(transform);
    tf_broadcaster_->sendTransform(transform_stamped);
  }

  trajectory_msgs::msg::JointTrajectory command_trajectory() const { return command_trajectory_; }

 private:
  void ViewpointResultCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    command_trajectory_ = *msg;
    is_sub_result_ = true;
  }

  void CallEmptyService(rclcpp::Client<std_srvs::srv::Empty>::SharedPtr& client) {
    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result_future = client->async_send_request(empty_request);
    const rclcpp::Time start = rclcpp::Clock(RCL_ROS_TIME).now();
    while ((rclcpp::Clock(RCL_ROS_TIME).now() - start).seconds() < kTimeout) {
      SpinOnce();
      auto status = result_future.wait_for(std::chrono::milliseconds(10));
      if (status == std::future_status::ready) {
        return;
      }
    }
    throw std::runtime_error("Can not call start service.");
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr base_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr viewpoint_result_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_service_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_service_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr set_viewpoint_mode_path_service_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr set_viewpoint_mode_tracking_service_client_;

  bool is_sub_result_;
  trajectory_msgs::msg::JointTrajectory command_trajectory_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

/// Class to test the viewpoint control node
class ViewpointControllerTest : public testing::Test {
 public:
  ViewpointControllerTest() {}
  virtual ~ViewpointControllerTest() = default;

 protected:
  virtual void SetUp() {
    // Generate test node
    test_node_ = std::make_shared<TestNode>();
    test_node_->Init();
    // Wait until linked with publisher and subscriber
    if (!test_node_->WaitForConnectionEstablished()) {
      RCLCPP_FATAL(rclcpp::get_logger("viewpoint_controller_test"), "Can not link to test target.");
      exit(EXIT_FAILURE);
    }
  }

  virtual void TearDown() {
    test_node_->ClearPath();
    const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
    // Wait until results stop coming
    while (test_node_->WaitForResult(kNoResultTimeout)) {
      if (rclcpp::Clock(RCL_ROS_TIME).now() - start_time > rclcpp::Duration::from_seconds(kTimeout)) {
        RCLCPP_FATAL(rclcpp::get_logger("viewpoint_controller_test"), "Can not clear path.");
        exit(EXIT_FAILURE);
      }
    }
  }
  // Test node
  std::shared_ptr<TestNode> test_node_;
};

// Normal case: Straight path
TEST_F(ViewpointControllerTest, StraightPath) {
  tf2::Transform robot_pose;
  sensor_msgs::msg::JointState joint_states;
  nav_msgs::msg::Path path;

  // Publish initial neck pan angle (0.0)
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Publish 2m straight path
  CreateStraightPath(path);
  test_node_->PublishBasePath(path);
  // Publish start position (0.0, 0.0)
  CreateGlobalPoseOrigin(robot_pose);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Wait for result (neck pan angle)
  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  // Verify result
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], 0.0, kThreshEqual);
  // Publish neck pan angle after moving forward (0.0)
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Move forward
  CreateGlobalPosePosition(robot_pose, kForwardX, 0.0);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], 0.0, kThreshEqual);
}

// Normal case: Right curve path
TEST_F(ViewpointControllerTest, RightCurvePath) {
  tf2::Transform robot_pose;
  sensor_msgs::msg::JointState joint_states;
  nav_msgs::msg::Path path;

  // Publish right curve path
  CreateRightCurvePath(path);
  test_node_->PublishBasePath(path);
  // Publish start position (0.0, 0.0)
  CreateGlobalPoseOrigin(robot_pose);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Publish initial neck pan angle (0.0)
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Wait for result (neck pan angle)
  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  // Verify result
  double path_angle = atan2(path.poses[2].pose.position.y - path.poses[1].pose.position.y,
                            path.poses[2].pose.position.x - path.poses[1].pose.position.x);
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], path_angle, kThreshEqual);
  // Move forward
  CreateGlobalPosePosition(robot_pose, kForwardX, 0.0);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Publish neck pan angle after moving forward
  CreateNeckPose(joint_states, Deg2Rad(kNeckRightDeg));
  test_node_->PublishJointStates(joint_states);

  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  path_angle = atan2(path.poses[3].pose.position.y - path.poses[2].pose.position.y,
                     path.poses[3].pose.position.x - path.poses[2].pose.position.x);
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], path_angle, kThreshEqual);
}

// Normal case: Left curve path
TEST_F(ViewpointControllerTest, LeftCurvePath) {
  tf2::Transform robot_pose;
  sensor_msgs::msg::JointState joint_states;
  nav_msgs::msg::Path path;

  // Publish left curve path
  CreateLeftCurvePath(path);
  test_node_->PublishBasePath(path);
  // Publish start position (0.0, 0.0)
  CreateGlobalPoseOrigin(robot_pose);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Publish initial neck pan angle (0.0)
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Wait for result (neck pan angle)
  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  // Verify result
  double path_angle = atan2(path.poses[2].pose.position.y - path.poses[1].pose.position.y,
                            path.poses[2].pose.position.x - path.poses[1].pose.position.x);
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], path_angle, kThreshEqual);
  // Move forward
  CreateGlobalPosePosition(robot_pose, kForwardX, 0.0);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Publish neck pan angle after moving forward
  CreateNeckPose(joint_states, Deg2Rad(kNeckLeftDeg));
  test_node_->PublishJointStates(joint_states);

  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  path_angle = atan2(path.poses[3].pose.position.y - path.poses[2].pose.position.y,
                     path.poses[3].pose.position.x - path.poses[2].pose.position.x);
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], path_angle, kThreshEqual);
}

// Normal case: Right diagonal path
TEST_F(ViewpointControllerTest, RightSlopePath) {
  tf2::Transform robot_pose;
  sensor_msgs::msg::JointState joint_states;
  nav_msgs::msg::Path path;

  // Publish right diagonal path
  CreateRightSlopePath(path);
  test_node_->PublishBasePath(path);
  // Publish start position (0.0, 0.0)
  CreateGlobalPoseOrigin(robot_pose);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Publish initial neck pan angle (0.0)
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Wait for result (neck pan angle)
  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  // Verify result
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], -kMaxRotationOnceRad, kThreshEqual);
  // Turn right
  CreateGlobalPoseTurn(robot_pose, Deg2Rad(kTurnRightDeg));
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Publish neck pan angle after turning
  CreateNeckPose(joint_states, Deg2Rad(kNeckRightDeg));
  test_node_->PublishJointStates(joint_states);
  // Verify result
  double path_angle = atan2(path.poses[2].pose.position.y - path.poses[1].pose.position.y,
                            path.poses[2].pose.position.x - path.poses[1].pose.position.x);
  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0],
              path_angle - Deg2Rad(kTurnRightDeg), kThreshEqual);
}

// Normal case: Left diagonal path
TEST_F(ViewpointControllerTest, LeftSlopePath) {
  tf2::Transform robot_pose;
  sensor_msgs::msg::JointState joint_states;
  nav_msgs::msg::Path path;

  // Publish left diagonal path
  CreateLeftSlopePath(path);
  test_node_->PublishBasePath(path);
  // Publish start position (0.0, 0.0)
  CreateGlobalPoseOrigin(robot_pose);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Publish initial neck pan angle (0.0)
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Wait for result (neck pan angle)
  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  // Verify result
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], kMaxRotationOnceRad, kThreshEqual);
  // Turn left
  CreateGlobalPoseTurn(robot_pose, Deg2Rad(kTurnLeftDeg));
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Publish neck pan angle after turning
  CreateNeckPose(joint_states, Deg2Rad(kNeckLeftDeg));
  test_node_->PublishJointStates(joint_states);

  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  double path_angle = atan2(path.poses[2].pose.position.y - path.poses[1].pose.position.y,
                            path.poses[2].pose.position.x - path.poses[1].pose.position.x);
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0],
              path_angle - Deg2Rad(kTurnLeftDeg), kThreshEqual);
}

// Normal case: Backward path
TEST_F(ViewpointControllerTest, BackPath) {
  tf2::Transform robot_pose;
  sensor_msgs::msg::JointState joint_states;
  nav_msgs::msg::Path path;

  // Publish backward path
  CreateBackPath(path);
  test_node_->PublishBasePath(path);
  // Publish start position (0.0, 0.0)
  CreateGlobalPoseOrigin(robot_pose);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Publish initial neck pan angle (0.0)
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Wait for result (neck pan angle)
  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  // Verify result
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], kMaxRotationOnceRad, kThreshEqual);
  // Turn left
  CreateGlobalPoseTurn(robot_pose, Deg2Rad(kTurnLeftDeg));
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Publish neck pan angle after turning
  CreateNeckPose(joint_states, Deg2Rad(kNeckLeftDeg));
  test_node_->PublishJointStates(joint_states);

  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  // Ensure the angle is rotated by the maximum rotation angle relative to the current neck pan angle
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0],
              kMaxRotationOnceRad + Deg2Rad(kNeckLeftDeg), kThreshEqual);
}

// Normal case: Near goal
TEST_F(ViewpointControllerTest, AroundGoal) {
  tf2::Transform robot_pose;
  sensor_msgs::msg::JointState joint_states;
  nav_msgs::msg::Path path;
  // Publish left diagonal path
  CreateLeftSlopePath(path);
  test_node_->PublishBasePath(path);
  // Publish start position (kNearGoalX, kNearGoalY)
  CreateGlobalPosePosition(robot_pose, kNearGoalX, kNearGoalY);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Wait for result (neck pan angle)
  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  // Verify result
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], kMaxRotationOnceRad, kThreshEqual);

  // Publish left diagonal path
  CreateLeftSlopePath(path);
  test_node_->PublishBasePath(path);
  // Publish position (kCloseGoalX, kCloseGoalY)
  CreateGlobalPosePosition(robot_pose, kCloseGoalX, kCloseGoalY);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Wait for result (neck pan angle)
  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  // Verify result
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], 0.0, kThreshEqual);
}

// Normal case: Check if the On/Off service for functionality works
TEST_F(ViewpointControllerTest, EnableService) {
  tf2::Transform robot_pose;
  sensor_msgs::msg::JointState joint_states;
  nav_msgs::msg::Path path;

  // Turn Off
  test_node_->CallStopService();
  CreateStraightPath(path);
  test_node_->PublishBasePath(path);
  CreateGlobalPoseOrigin(robot_pose);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Confirm that no results are published
  ASSERT_FALSE(test_node_->WaitForResult(kNoResultTimeout));

  // Turn On
  test_node_->CallStartService();
  CreateStraightPath(path);
  test_node_->PublishBasePath(path);
  CreateGlobalPoseOrigin(robot_pose);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Wait for result (neck pan angle)
  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  // Verify result
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], 0.0, kThreshEqual);
}

// Normal case: Check if the service to switch to tracking target mode works
TEST_F(ViewpointControllerTest, TrackingMode) {
  tf2::Transform robot_pose;
  sensor_msgs::msg::JointState joint_states;
  nav_msgs::msg::Path path;
  nav_msgs::msg::Path target_path;

  // Switch to tracking target mode
  test_node_->CallSetViewpointTrackingTargetService();
  // Movement path is left curve, target is right curve
  CreateLeftCurvePath(path);
  test_node_->PublishBasePath(path);
  CreateRightCurvePath(target_path);
  test_node_->PublishTargetPath(target_path);

  // Publish start position (0.0, 0.0)
  CreateGlobalPoseOrigin(robot_pose);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Publish initial neck pan angle (0.0)
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Wait for result (neck pan angle)
  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  // Verify result: Confirm that it is in the direction of the latest target position
  double target_angle = atan2(target_path.poses.back().pose.position.y, target_path.poses.back().pose.position.x);
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], target_angle, kThreshEqual);
  // Switch back to movement path mode
  test_node_->CallSetViewpointModePathService();
}

/// Normal case: In tracking target mode, check if the viewpoint does not move when the target is not being tracked
/// Does not move the viewpoint
TEST_F(ViewpointControllerTest, TrackingModeLost) {
  tf2::Transform robot_pose;
  sensor_msgs::msg::JointState joint_states;
  nav_msgs::msg::Path path;

  // Switch to tracking target mode
  test_node_->CallSetViewpointTrackingTargetService();
  // Movement path is left curve, tracking target is not published
  CreateLeftCurvePath(path);
  test_node_->PublishBasePath(path);

  // Publish start position (0.0, 0.0)
  CreateGlobalPoseOrigin(robot_pose);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Publish initial neck pan angle (0.0)
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Confirm that no results are published
  ASSERT_FALSE(test_node_->WaitForResult(kNoResultTimeout));
  // Switch back to movement path mode
  test_node_->CallSetViewpointModePathService();
}

// Normal case: Check if the service to switch to movement path mode works
TEST_F(ViewpointControllerTest, PathMode) {
  tf2::Transform robot_pose;
  sensor_msgs::msg::JointState joint_states;
  nav_msgs::msg::Path path;
  nav_msgs::msg::Path target_path;

  // Switch to tracking target mode
  test_node_->CallSetViewpointTrackingTargetService();
  // Switch to movement path mode
  test_node_->CallSetViewpointModePathService();
  // Movement path is left curve, target is right curve
  CreateLeftCurvePath(path);
  test_node_->PublishBasePath(path);
  CreateRightCurvePath(target_path);
  test_node_->PublishTargetPath(target_path);

  // Publish start position (0.0, 0.0)
  CreateGlobalPoseOrigin(robot_pose);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  // Publish initial neck pan angle (0.0)
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Wait for result (neck pan angle)
  ASSERT_TRUE(test_node_->WaitForResult(kTimeout));
  // Verify result: Confirm it is on the left curve side
  double path_angle = atan2(path.poses[2].pose.position.y - path.poses[1].pose.position.y,
                            path.poses[2].pose.position.x - path.poses[1].pose.position.x);
  EXPECT_NEAR(test_node_->command_trajectory().points[0].positions[0], path_angle, kThreshEqual);
}

// Abnormal case: 0 path
TEST_F(ViewpointControllerTest, NoPath) {
  tf2::Transform robot_pose;
  sensor_msgs::msg::JointState joint_states;
  nav_msgs::msg::Path path;

  // Publish 0 path
  CreateZeroPath(path);
  test_node_->PublishBasePath(path);
  CreateGlobalPoseOrigin(robot_pose);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Confirm that no results are published
  ASSERT_FALSE(test_node_->WaitForResult(kNoResultTimeout));
}

// Abnormal case: When the robot and the path are far apart
TEST_F(ViewpointControllerTest, ExeedLimitNearestDistance) {
  tf2::Transform robot_pose;
  sensor_msgs::msg::JointState joint_states;
  nav_msgs::msg::Path path;

  CreateStraightPath(path);
  test_node_->PublishBasePath(path);
  // Publish start position (kBackX, 0.0)
  CreateGlobalPosePosition(robot_pose, kBackX, 0.0);
  test_node_->SendTransform(robot_pose, "map", "base_footprint");
  CreateNeckPose(joint_states, 0.0);
  test_node_->PublishJointStates(joint_states);
  // Confirm that no results are published
  ASSERT_FALSE(test_node_->WaitForResult(kNoResultTimeout));
}
}  // namespace tmc_viewpoint_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);
  // Generate viewpoint_controller node
  auto viewpoint_controller_node = std::make_shared<tmc_viewpoint_controller::ViewpointControllerNode>(option);
  viewpoint_controller_node->Init();
  // Create a thread to run
  auto viewpoint_controller_node_thread = std::make_shared<std::thread>([&]() {
      viewpoint_controller_node->Run();
      });
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  viewpoint_controller_node_thread->join();
  viewpoint_controller_node.reset();

  return result;
}
