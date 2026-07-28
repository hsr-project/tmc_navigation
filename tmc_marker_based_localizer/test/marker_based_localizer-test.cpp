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

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "marker_based_localizer-test_common.hpp"
#include "marker_based_localizer-test_node.hpp"

namespace tmc_marker_based_localizer {
/// Normal case
/// Self-position correction can be performed using markers
TEST_F(MarkerBasedLocalizerNodeTest, LocalizeBasedOnMarker) {
  // parameter
  // Information about the marker recognized by the camera
  // If the distance is greater than marker_to_base_distance_threshold (def 1.5), the process will not be executed
  const int object_id = 507;
  const geometry_msgs::msg::Pose camera_marker_object_pose = CreatePose(0.65, 0.86, 1.12, 0.48, -0.43, 0.54, 0.53);
  // tf from cart to camera
  const tf2::Transform robot_pose = CreateTransform(0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 1.0);
  // If joint_stopping_vel (def:0.005) or less, it is considered stopped
  const std::string joint_state_name = "left_drive_wheel_joint";
  const double joint_state_velocity = 0.005;
  // If the movement of the odom x, y axes is less than travel_distance_threshold (def:5.0), the process will not be executed
  const double odom_x_init = 0.0;
  const double odom_y_init = 0.0;
  const double odom_x = 5.0;
  const double odom_y = 0.0;
  geometry_msgs::msg::TransformStamped transform_stamped;
  geometry_msgs::msg::Pose localized_pose;

  // exercise
  // Initial odometry transmission
  test_node_->PublishOdometry(odom_x_init, odom_y_init);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish joint_state
  test_node_->PublishJointState(joint_state_name, joint_state_velocity);
  // Update odometry
  test_node_->PublishOdometry(odom_x, odom_y);
  // Publish tf
  test_node_->SendTransformStatic(robot_pose, kDefaultBaseTfName, kCameraTfName, transform_stamped);
  // Generate expected value
  test_node_->CreateExpectMarkerPose(object_id, camera_marker_object_pose, transform_stamped, localized_pose);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish marker
  test_node_->PublishMarker(object_id, camera_marker_object_pose);

  // verify
  ASSERT_TRUE(test_node_->WaitForResult(kNoResultTimeout));
  EXPECT_EQ(test_node_->localized_pose()->pose.pose, localized_pose);
}

/// Self-position correction can be performed using a marker other than "LocalizeBasedOnMarker"
TEST_F(MarkerBasedLocalizerNodeTest, LocalizeBasedOnOtherMarker) {
  // parameter
  const int object_id = 508;
  const geometry_msgs::msg::Pose camera_marker_object_pose = CreatePose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const tf2::Transform robot_pose = CreateTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const std::string joint_state_name = "left_drive_wheel_joint";
  const double joint_state_velocity = 0.005;
  const double odom_x_init = 0.0;
  const double odom_y_init = 0.0;
  const double odom_x = 5.0;
  const double odom_y = 0.0;
  geometry_msgs::msg::TransformStamped transform_stamped;
  geometry_msgs::msg::Pose localized_pose;

  // exercise
  // Initial odometry transmission
  test_node_->PublishOdometry(odom_x_init, odom_y_init);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish joint_state
  test_node_->PublishJointState(joint_state_name, joint_state_velocity);
  // Update odometry
  test_node_->PublishOdometry(odom_x, odom_y);
  // Publish tf
  test_node_->SendTransformStatic(robot_pose, kDefaultBaseTfName, kCameraTfName, transform_stamped);
  // Generate expected value
  test_node_->CreateExpectMarkerPose(object_id, camera_marker_object_pose, transform_stamped, localized_pose);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish marker
  test_node_->PublishMarker(object_id, camera_marker_object_pose);

  // verify
  ASSERT_TRUE(test_node_->WaitForResult(kNoResultTimeout));
  EXPECT_EQ(test_node_->localized_pose()->pose.pose, localized_pose);
}

/// Self-position estimation using markers can be toggled ON and OFF via a service
TEST_F(MarkerBasedLocalizerNodeTest, SwitchMarkerBasedLocalizerService) {
  // parameter
  const int object_id = 507;
  const geometry_msgs::msg::Pose camera_marker_object_pose = CreatePose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const tf2::Transform robot_pose = CreateTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const std::string joint_state_name = "left_drive_wheel_joint";
  const double joint_state_velocity = 0.005;
  const double odom_x_init = 0.0;
  const double odom_y_init = 0.0;
  const double odom_x = 5.0;
  const double odom_y = 0.0;
  geometry_msgs::msg::TransformStamped transform_stamped;
  geometry_msgs::msg::Pose localized_pose;

  // exercise
  // By default, the self-position correction function using markers is ON, so start testing with the function turned OFF via the service
  // Stop the service
  test_node_->CallStopService();
  // Initial odometry transmission
  test_node_->PublishOdometry(odom_x_init, odom_y_init);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish joint_state
  test_node_->PublishJointState(joint_state_name, joint_state_velocity);
  // Update odometry
  test_node_->PublishOdometry(odom_x, odom_y);
  // Publish tf
  test_node_->SendTransformStatic(robot_pose, kDefaultBaseTfName, kCameraTfName, transform_stamped);
  // Generate expected value
  test_node_->CreateExpectMarkerPose(object_id, camera_marker_object_pose, transform_stamped, localized_pose);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish marker
  test_node_->PublishMarker(object_id, camera_marker_object_pose);

  // verify
  ASSERT_FALSE(test_node_->WaitForResult(kNoResultTimeout));

  // exercise
  // Execute the service
  test_node_->CallStartService();
  // Initial odometry transmission
  test_node_->PublishOdometry(odom_x_init, odom_y_init);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish joint_state
  test_node_->PublishJointState(joint_state_name, joint_state_velocity);
  // Update odometry
  test_node_->PublishOdometry(odom_x, odom_y);
  // Publish tf
  test_node_->SendTransformStatic(robot_pose, kDefaultBaseTfName, kCameraTfName, transform_stamped);
  // Generate expected value
  test_node_->CreateExpectMarkerPose(object_id, camera_marker_object_pose, transform_stamped, localized_pose);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish marker
  test_node_->PublishMarker(object_id, camera_marker_object_pose);

  // verify
  ASSERT_TRUE(test_node_->WaitForResult(kNoResultTimeout));
  EXPECT_EQ(test_node_->localized_pose()->pose.pose, localized_pose);
}

/// Semi-normal case
/// If there is no matching object_id, self-position correction is not performed
TEST_F(MarkerBasedLocalizerNodeTest, NotLocalizeForNoObjctId) {
  // parameter
  const int object_id = 506;
  const geometry_msgs::msg::Pose camera_marker_object_pose = CreatePose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const tf2::Transform robot_pose = CreateTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const std::string joint_state_name = "left_drive_wheel_joint";
  const double joint_state_velocity = 0.005;
  const double odom_x_init = 0.0;
  const double odom_y_init = 0.0;
  const double odom_x = 5.0;
  const double odom_y = 0.0;
  geometry_msgs::msg::TransformStamped transform_stamped;
  geometry_msgs::msg::Pose localized_pose;

  // exercise
  // Initial odometry transmission
  test_node_->PublishOdometry(odom_x_init, odom_y_init);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish joint_state
  test_node_->PublishJointState(joint_state_name, joint_state_velocity);
  // Update odometry
  test_node_->PublishOdometry(odom_x, odom_y);
  // Publish tf
  test_node_->SendTransformStatic(robot_pose, kDefaultBaseTfName, kCameraTfName, transform_stamped);
  // Generate expected value
  test_node_->CreateExpectMarkerPose(object_id, camera_marker_object_pose, transform_stamped, localized_pose);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish marker
  test_node_->PublishMarker(object_id, camera_marker_object_pose);

  // verify
  ASSERT_FALSE(test_node_->WaitForResult(kNoResultTimeout));
}

/// If not stopped, self-position correction is not performed
TEST_F(MarkerBasedLocalizerNodeTest, NotLocalizeForNoStop) {
  // parameter
  const int object_id = 507;
  const geometry_msgs::msg::Pose camera_marker_object_pose = CreatePose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const tf2::Transform robot_pose = CreateTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const std::string joint_state_name = "left_drive_wheel_joint";
  const double joint_state_velocity = 0.1;
  const double odom_x_init = 0.0;
  const double odom_y_init = 0.0;
  const double odom_x = 5.0;
  const double odom_y = 0.0;
  geometry_msgs::msg::TransformStamped transform_stamped;
  geometry_msgs::msg::Pose localized_pose;

  // exercise
  // Initial odometry transmission
  test_node_->PublishOdometry(odom_x_init, odom_y_init);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish joint_state
  test_node_->PublishJointState(joint_state_name, joint_state_velocity);
  // Update odometry
  test_node_->PublishOdometry(odom_x, odom_y);
  // Publish tf
  test_node_->SendTransformStatic(robot_pose, kDefaultBaseTfName, kCameraTfName, transform_stamped);
  // Generate expected value
  test_node_->CreateExpectMarkerPose(object_id, camera_marker_object_pose, transform_stamped, localized_pose);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish marker
  test_node_->PublishMarker(object_id, camera_marker_object_pose);

  // verify
  ASSERT_FALSE(test_node_->WaitForResult(kNoResultTimeout));
}

/// If the marker has not moved from the previously recognized location, self-position correction is not performed
TEST_F(MarkerBasedLocalizerNodeTest, NotLocalizeForNoMoveFromPrevLocalize) {
  // parameter
  const int object_id = 507;
  const geometry_msgs::msg::Pose camera_marker_object_pose = CreatePose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const tf2::Transform robot_pose = CreateTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const std::string joint_state_name = "left_drive_wheel_joint";
  const double joint_state_velocity = 0.005;
  const double odom_x_init = 0.0;
  const double odom_y_init = 0.0;
  const double odom_x = 5.0;
  const double odom_y = 0.0;
  geometry_msgs::msg::TransformStamped transform_stamped;
  geometry_msgs::msg::Pose localized_pose;

  // Successfully identify the marker once and update the previous identification location
  // exercise
  // Initial odometry transmission
  test_node_->PublishOdometry(odom_x_init, odom_y_init);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish joint_state
  test_node_->PublishJointState(joint_state_name, joint_state_velocity);
  // Update odometry
  test_node_->PublishOdometry(odom_x, odom_y);
  // Publish tf
  test_node_->SendTransformStatic(robot_pose, kDefaultBaseTfName, kCameraTfName, transform_stamped);
  // Generate expected value
  test_node_->CreateExpectMarkerPose(object_id, camera_marker_object_pose, transform_stamped, localized_pose);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish marker
  test_node_->PublishMarker(object_id, camera_marker_object_pose);

  // verify
  ASSERT_TRUE(test_node_->WaitForResult(kNoResultTimeout));

  // Identify the marker at the same location as the previous identification
  // Publish joint_state
  test_node_->PublishJointState(joint_state_name, joint_state_velocity);
  // Update odometry
  test_node_->PublishOdometry(odom_x, odom_y);
  // Publish tf
  test_node_->SendTransformStatic(robot_pose, kDefaultBaseTfName, kCameraTfName, transform_stamped);
  // Generate expected value
  test_node_->CreateExpectMarkerPose(object_id, camera_marker_object_pose, transform_stamped, localized_pose);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish marker
  test_node_->PublishMarker(object_id, camera_marker_object_pose);

  // verify
  ASSERT_FALSE(test_node_->WaitForResult(kNoResultTimeout));
}

/// If the distance from the marker is too far, self-position correction is not performed
TEST_F(MarkerBasedLocalizerNodeTest, NotLocalizeForFarMarker) {
  // parameter
  const int object_id = 507;
  const geometry_msgs::msg::Pose camera_marker_object_pose = CreatePose(2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const tf2::Transform robot_pose = CreateTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const std::string joint_state_name = "left_drive_wheel_joint";
  const double joint_state_velocity = 0.005;
  const double odom_x_init = 0.0;
  const double odom_y_init = 0.0;
  const double odom_x = 5.0;
  const double odom_y = 0.0;
  geometry_msgs::msg::TransformStamped transform_stamped;
  geometry_msgs::msg::Pose localized_pose;

  // exercise
  // Initial odometry transmission
  test_node_->PublishOdometry(odom_x_init, odom_y_init);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish joint_state
  test_node_->PublishJointState(joint_state_name, joint_state_velocity);
  // Update odometry
  test_node_->PublishOdometry(odom_x, odom_y);
  // Publish tf
  test_node_->SendTransformStatic(robot_pose, kDefaultBaseTfName, kCameraTfName, transform_stamped);
  // Generate expected value
  test_node_->CreateExpectMarkerPose(object_id, camera_marker_object_pose, transform_stamped, localized_pose);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish marker
  test_node_->PublishMarker(object_id, camera_marker_object_pose);

  // verify
  ASSERT_FALSE(test_node_->WaitForResult(kNoResultTimeout));
}

/// Abnormal case
/// If there is no matching JointState, output an error and do not perform self-position correction
TEST_F(MarkerBasedLocalizerNodeTest, NotLocalizeForNoJointState) {
  // parameter
  const int object_id = 507;
  const geometry_msgs::msg::Pose camera_marker_object_pose = CreatePose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const tf2::Transform robot_pose = CreateTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const double odom_x_init = 0.0;
  const double odom_y_init = 0.0;
  const double odom_x = 5.0;
  const double odom_y = 0.0;
  geometry_msgs::msg::TransformStamped transform_stamped;
  geometry_msgs::msg::Pose localized_pose;

  // exercise
  // Initial odometry transmission
  test_node_->PublishOdometry(odom_x_init, odom_y_init);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish joint_state with a dummy axis name
  test_node_->PublishDummyJointState();
  // Update odometry
  test_node_->PublishOdometry(odom_x, odom_y);
  // Publish tf
  test_node_->SendTransformStatic(robot_pose, kDefaultBaseTfName, kCameraTfName, transform_stamped);
  // Generate expected value
  test_node_->CreateExpectMarkerPose(object_id, camera_marker_object_pose, transform_stamped, localized_pose);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish marker
  test_node_->PublishMarker(object_id, camera_marker_object_pose);

  // verify
  ASSERT_FALSE(test_node_->WaitForResult(kNoResultTimeout));
}

/// If tf conversion fails, output an error and do not perform self-position correction
TEST_F(MarkerBasedLocalizerNodeTest, NotLocalizeForCannotTransform) {
  // parameter
  const int object_id = 507;
  const geometry_msgs::msg::Pose camera_marker_object_pose = CreatePose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const tf2::Transform robot_pose = CreateTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const std::string joint_state_name = "left_drive_wheel_joint";
  const double joint_state_velocity = 0.005;
  const double odom_x_init = 0.0;
  const double odom_y_init = 0.0;
  const double odom_x = 5.0;
  const double odom_y = 0.0;
  geometry_msgs::msg::TransformStamped transform_stamped;
  geometry_msgs::msg::Pose localized_pose;

  // exercise
  // Initial odometry transmission
  test_node_->PublishOdometry(odom_x_init, odom_y_init);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish joint_state
  test_node_->PublishJointState(joint_state_name, joint_state_velocity);
  // Update odometry
  test_node_->PublishOdometry(odom_x, odom_y);
  // Publish tf (make it abnormal by publishing with tf)
  test_node_->SendTransform(robot_pose, kDefaultBaseTfName, kCameraTfName, transform_stamped);
  // Generate expected value
  test_node_->CreateExpectMarkerPose(object_id, camera_marker_object_pose, transform_stamped, localized_pose);
  // Wait for the above transmission to complete
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(kReceiveWaitTime * 1000)));
  // Publish marker
  test_node_->PublishMarker(object_id, camera_marker_object_pose);

  // verify
  ASSERT_FALSE(test_node_->WaitForResult(kNoResultTimeout));
}
}  // namespace tmc_marker_based_localizer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);

  // Generate marker_based_localizer node
  auto marker_based_localizer_node = std::make_shared<tmc_marker_based_localizer::MarkerBasedLocalizerNode>(option);
  // Read parameters from yaml
  const std::string yaml_directory = ament_index_cpp::get_package_share_directory("tmc_marker_based_localizer") +
                                     "/test/parameter/";
  tmc_marker_based_localizer::LoadParameterFromYaml(marker_based_localizer_node,
                                                    yaml_directory, "marker_based_localizer-test.yaml");

  // Since the marker_based_localizer node waits to receive the "joint_state" topic during Init, create a node to publish "joint_state"
  auto pub_joint_state_test_node_ = std::make_shared<tmc_marker_based_localizer::PubJointStateTestNode>(option);
  pub_joint_state_test_node_->Init();
  // Start a thread to publish "joint_state"
  auto pub_joint_state_test_node_thread = std::make_shared<std::thread>([&]() {
    pub_joint_state_test_node_->PublishJointState();
  });

  marker_based_localizer_node->Init();

  // Stop publishing "joint_state" once the Init of the marker_based_localizer node is complete
  pub_joint_state_test_node_->StopPublishJointState();
  pub_joint_state_test_node_thread->join();
  pub_joint_state_test_node_.reset();

  // Start a thread to spin
  auto marker_based_localizer_node_thread = std::make_shared<std::thread>([&]() {
    try {
      rclcpp::spin(marker_based_localizer_node);
    } catch (const std::exception& e) {
      // Exceptions may occur during the termination process
      std::cout << "marker_based_localizer_node : " << e.what() << std::endl;
    }
  });
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  marker_based_localizer_node_thread->join();
  marker_based_localizer_node.reset();

  return result;
}
