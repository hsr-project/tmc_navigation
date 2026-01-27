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

#include "marker_based_localizer-test_node.hpp"

#include <chrono>

namespace {
// Topic name
const char* kMarkerTopicName = "recognized_object";
const char* kOdometryTopicName = "odom";
const char* kLocalizedPoseTopicName = "laser_2d_correct_pose";
const char* kJointStateTopicName = "joint_states";
/// Topic buffer size
const uint32_t kTopicBufferSize = 1;
// Timeout duration [s]
double kTimeout = 5.0;
}  // unnamed namespace

namespace tmc_marker_based_localizer {
/// PubJointStateTestNode
using std::placeholders::_1;
/// Initialization
void PubJointStateTestNode::Init() {
  // Publisher settings
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(kJointStateTopicName, kTopicBufferSize);
  // Generate joint_state
  joint_state_ = CreateJointState({"left_drive_wheel_joint"});
  joint_state_pub_flg_ = true;
}

// Publish joint_state
void PubJointStateTestNode::PublishJointState() {
  rclcpp::Rate rate(kRate);
  while (joint_state_pub_flg_) {
    joint_state_pub_->publish(joint_state_);
    rate.sleep();
  }
}

// Stop publishing data
void PubJointStateTestNode::StopPublishJointState() {
  joint_state_pub_flg_ = false;
}


/// TestNode
/// Initialization
void TestNode::Init() {
  // Publisher settings
  marker_pub_ = this->create_publisher<tmc_vision_msgs::msg::RecognizedObject>(kMarkerTopicName, kTopicBufferSize);
  odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(kOdometryTopicName, kTopicBufferSize);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(kJointStateTopicName, kTopicBufferSize);

  // Subscriber settings
  localized_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      kLocalizedPoseTopicName, kTopicBufferSize, std::bind(&TestNode::LocalizedPoseSubscriptionCallback_, this, _1));

  // Service settings
  start_service_client_ =
      this->create_client<std_srvs::srv::Empty>("/marker_based_localizer/start_marker_based_localizer");
  stop_service_client_ =
      this->create_client<std_srvs::srv::Empty>("/marker_based_localizer/stop_marker_based_localizer");

  // Retrieve parameter
  // Service settings
  rclcpp::AsyncParametersClient::SharedPtr param_client =
      std::make_shared<rclcpp::AsyncParametersClient>(this, "/marker_based_localizer");
  if (!param_client->wait_for_service(std::chrono::milliseconds(static_cast<int32_t>(kTimeout * 1000)))) {
    RCLCPP_ERROR(this->get_logger(), "Parameter service error.");
    return;
  }
  // Retrieve parameter list
  auto get_param_list_future = param_client->list_parameters({}, 0);
  if (rclcpp::spin_until_future_complete(shared_from_this(), get_param_list_future,
      std::chrono::milliseconds(static_cast<int32_t>(kTimeout * 1000))) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to get parameter list.");
    return;
  }
  auto responese_param_list = get_param_list_future.get();
  // Retrieve parameter
  auto get_param_future = param_client->get_parameters(responese_param_list.names);
  if (rclcpp::spin_until_future_complete(shared_from_this(), get_param_future,
      std::chrono::milliseconds(static_cast<int32_t>(kTimeout * 1000))) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to get parameter.");
    return;
  }
  std::vector<rclcpp::Parameter> responese_param = get_param_future.get();
  for (size_t i = 0; i < responese_param.size(); i++) {
    if (responese_param[i].get_name().find("marker_objects") != std::string::npos) {
      marker_objects_param_.emplace(responese_param[i].get_name(), responese_param[i]);
    } else if (responese_param[i].get_name().find("joints_list") != std::string::npos) {
      joints_list_ = responese_param[i].as_string_array();
    }
  }
}

// Wait until connection with target node is established
bool TestNode::WaitForConnectionEstablished() {
  const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
  while ((marker_pub_->get_subscription_count() == 0 ||
          odometry_pub_->get_subscription_count() == 0 ||
          joint_state_pub_->get_subscription_count() == 0 ||
          localized_pose_sub_->get_publisher_count() == 0)) {
    if ((rclcpp::Clock(RCL_ROS_TIME).now() - start_time) > rclcpp::Duration::from_seconds(kTimeout)) {
      RCLCPP_FATAL(this->get_logger(), "Can not link to target.");
      return false;
    }
    SpinOnce();
    rate_.sleep();
  }
  return true;
}

bool TestNode::WaitForResult(const double timeout) {
  const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
  while (!is_sub_result_ &&
         rclcpp::Clock(RCL_ROS_TIME).now() - start_time < rclcpp::Duration::from_seconds(timeout)) {
    SpinOnce();
    rate_.sleep();
  }
  return is_sub_result_;
}

// Spin test node
void TestNode::SpinOnce() {
  rclcpp::spin_some(shared_from_this());
}

// Start Service
void TestNode::CallStartService() {
  CallEmptyService(start_service_client_);
}

// Stop Service
void TestNode::CallStopService() {
  CallEmptyService(stop_service_client_);
}

// Publish Marker
void TestNode::PublishMarker(const int object_id, const geometry_msgs::msg::Pose& object_pose) {
  tmc_vision_msgs::msg::RecognizedObject marker;
  marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  marker.header.frame_id = kCameraTfName;
  marker.object_id.object_id = object_id;
  marker.object_frame = object_pose;
  is_sub_result_ = false;
  marker_pub_->publish(marker);
}

// Publish Odometry
void TestNode::PublishOdometry(const double odom_x, const double odom_y) {
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  odometry.pose.pose.position.x = odom_x;
  odometry.pose.pose.position.y = odom_y;
  odometry_pub_->publish(odometry);
}

// Publish joint_state
void TestNode::PublishJointState(const std::string& joint_name, const double velocity) {
  sensor_msgs::msg::JointState joint_state = CreateJointState(joints_list_);
  SetJointStateVelocity(joint_state, joint_name, velocity);
  joint_state_pub_->publish(joint_state);
}

// Publish joint_state with dummy axis name
void TestNode::PublishDummyJointState() {
  sensor_msgs::msg::JointState joint_state = CreateJointState(joints_list_);
  joint_state.name[0] = joint_state.name[0] + "_dummy";
  joint_state_pub_->publish(joint_state);
}

// Send tf_static
void TestNode::SendTransformStatic(const tf2::Transform& transform, const std::string& frame_id,
                                   const std::string& child_frame_id,
                                   geometry_msgs::msg::TransformStamped& transform_stamped) {
  tf2_ros::StaticTransformBroadcaster tf_static_broadcaster(shared_from_this());
  transform_stamped.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform = tf2::toMsg(transform);
  tf_static_broadcaster.sendTransform(transform_stamped);
}

// Send tf
void TestNode::SendTransform(const tf2::Transform& transform, const std::string& frame_id,
                             const std::string& child_frame_id,
                             geometry_msgs::msg::TransformStamped& transform_stamped) {
  tf2_ros::TransformBroadcaster tf_broadcaster(shared_from_this());
  transform_stamped.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform = tf2::toMsg(transform);
  tf_broadcaster.sendTransform(transform_stamped);
}

void TestNode::LocalizedPoseSubscriptionCallback_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  localized_pose_ = msg;
  is_sub_result_ = true;
}

// Execute Service
void TestNode::CallEmptyService(rclcpp::Client<std_srvs::srv::Empty>::SharedPtr& client) {
  auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result_future = client->async_send_request(empty_request);
  const rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
  while ((rclcpp::Clock(RCL_ROS_TIME).now() - start_time) < rclcpp::Duration::from_seconds(kTimeout)) {
    SpinOnce();
    auto status = result_future.wait_for(std::chrono::milliseconds(10));
    if (status == std::future_status::ready) {
      return;
    }
    rate_.sleep();
  }
  throw std::runtime_error("Can not call service.");
}

// Set marker information
void TestNode::MarkerPoseInfo_(const int object_id, geometry_msgs::msg::Pose& object_pose) {
  for (auto iter = marker_objects_param_.begin(); iter != marker_objects_param_.end(); iter++) {
    if (iter->first.find("object_id") != std::string::npos) {
      if (marker_objects_param_[iter->first].as_int() == object_id) {
        std::string key_object_id = iter->first;
        const std::string key_object = key_object_id.erase(key_object_id.find("object_id") - 1);
        const std::vector<double> position = marker_objects_param_[key_object + ".translation"].as_double_array();
        const std::vector<double> orientation = marker_objects_param_[key_object + ".rotation"].as_double_array();
        object_pose.position.x = position[0];
        object_pose.position.y = position[1];
        object_pose.position.z = position[2];
        object_pose.orientation.x = orientation[0];
        object_pose.orientation.y = orientation[1];
        object_pose.orientation.z = orientation[2];
        object_pose.orientation.w = orientation[3];
      }
    }
  }
}

// Generate expected value
void TestNode::CreateExpectMarkerPose(const int object_id, const geometry_msgs::msg::Pose& camera_marker_object_pose,
                                      const geometry_msgs::msg::TransformStamped& transform_stamped,
                                      geometry_msgs::msg::Pose& localized_pose) {
  // Robot-marker position
  geometry_msgs::msg::Pose base_marker_object_pose;
  tf2::doTransform(camera_marker_object_pose, base_marker_object_pose, transform_stamped);
  Eigen::Affine3d base_to_marker;
  tmc_eigen_bridge::PoseMsgToAffine3d(base_marker_object_pose, base_to_marker);
  const Eigen::Affine3d marker_to_base = base_to_marker.inverse();

  // Map-marker position
  geometry_msgs::msg::Pose map_marker_object_pose;
  MarkerPoseInfo_(object_id, map_marker_object_pose);
  Eigen::Affine3d floor_to_marker;
  tmc_eigen_bridge::PoseMsgToAffine3d(map_marker_object_pose, floor_to_marker);
  const Eigen::Affine3d floor_to_base = floor_to_marker * marker_to_base;
  tmc_eigen_bridge::Affine3dToPoseMsg(floor_to_base, localized_pose);
}


/// MarkerBasedLocalizerNodeTest
void MarkerBasedLocalizerNodeTest::SetUp() {
  // Generate test node
  test_node_ = std::make_shared<TestNode>();
  test_node_->Init();
  // Wait until linked with publisher and subscriber
  if (!test_node_->WaitForConnectionEstablished()) {
    RCLCPP_FATAL(rclcpp::get_logger("marker_based_localizer_test"), "Can not link to test target.");
    exit(EXIT_FAILURE);
  }
}
}  // namespace tmc_marker_based_localizer
