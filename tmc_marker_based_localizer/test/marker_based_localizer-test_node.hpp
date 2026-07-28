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

#ifndef TMC_MARKER_BASED_LOCALIZER_MARKER_BASED_LOCALIZER_TEST_NODE_HPP_
#define TMC_MARKER_BASED_LOCALIZER_MARKER_BASED_LOCALIZER_TEST_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "../src/marker_based_localizer.hpp"
#include "marker_based_localizer-test_common.hpp"

namespace tmc_marker_based_localizer {
class PubJointStateTestNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit PubJointStateTestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node_pub_joint_state", options) {}

  /// Initialization
  void Init();
  // Publish joint_state
  void PublishJointState();
  // Stop publishing data
  void StopPublishJointState();

 private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  bool joint_state_pub_flg_;
  sensor_msgs::msg::JointState joint_state_;
};


class TestNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options), rate_(kRate) {}

  /// Initialization
  void Init();
  // Wait until the connection with the target node is established
  bool WaitForConnectionEstablished();
  bool WaitForResult(const double timeout);
  // Spin the test node
  void SpinOnce();
  // Provide subscribed data
  std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> localized_pose() const { return localized_pose_; }
  // Start the service
  void CallStartService();
  // Stop the service
  void CallStopService();
  // Publish Marker
  void PublishMarker(const int object_id, const geometry_msgs::msg::Pose& object_pose);
  // Publish Odometry
  void PublishOdometry(const double odom_x, const double odom_y);
  // Publish joint_state
  void PublishJointState(const std::string& joint_name, const double velocity);
  // Publish joint_state with dummy axis names
  void PublishDummyJointState();
  // Send tf_static
  void SendTransformStatic(const tf2::Transform& transform, const std::string& frame_id,
                           const std::string& child_frame_id,
                           geometry_msgs::msg::TransformStamped& transform_stamped);
  // Send tf
  void SendTransform(const tf2::Transform& transform, const std::string& frame_id,
                     const std::string& child_frame_id,
                     geometry_msgs::msg::TransformStamped& transform_stamped);
  // Generate expected values
  void CreateExpectMarkerPose(const int object_id, const geometry_msgs::msg::Pose& camera_marker_object_pose,
                              const geometry_msgs::msg::TransformStamped& transform_stamped,
                              geometry_msgs::msg::Pose& localized_pose);

 private:
  rclcpp::Publisher<tmc_vision_msgs::msg::RecognizedObject>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localized_pose_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_service_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_service_client_;
  std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> localized_pose_;
  bool is_sub_result_;
  rclcpp::Rate rate_;
  std::map<std::string, rclcpp::Parameter> marker_objects_param_;
  std::vector<std::string> joints_list_;

  void LocalizedPoseSubscriptionCallback_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  // Execute the service
  void CallEmptyService(rclcpp::Client<std_srvs::srv::Empty>::SharedPtr& client);
  // Set marker information
  void MarkerPoseInfo_(const int object_id, geometry_msgs::msg::Pose& object_pose);
};


class MarkerBasedLocalizerNodeTest : public testing::Test {
 public:
  /// Constructor
  MarkerBasedLocalizerNodeTest() {}
  /// Destructor
  ~MarkerBasedLocalizerNodeTest() {}

 protected:
  virtual void SetUp();
  virtual void TearDown() {}
  // Test node
  std::shared_ptr<TestNode> test_node_;
};
}  // namespace tmc_marker_based_localizer

#endif  // TMC_MARKER_BASED_LOCALIZER_MARKER_BASED_LOCALIZER_TEST_NODE_HPP_
