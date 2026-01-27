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
/// @file     marker_based_localizer.hpp
/// @brief    Self-position estimation by marker recognition
/// @author   Yoshiaki Asahara
/// @version  1.0.0

#ifndef TMC_MARKER_BASED_LOCALIZER_MARKER_BASED_LOCALIZER_HPP_
#define TMC_MARKER_BASED_LOCALIZER_MARKER_BASED_LOCALIZER_HPP_

#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tmc_eigen_bridge/eigen_bridge.hpp>
#include <tmc_vision_msgs/msg/recognized_object.hpp>

namespace tmc_marker_based_localizer {
class MarkerBasedLocalizerNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit MarkerBasedLocalizerNode(const rclcpp::NodeOptions& options);
  /// Destructor
  ~MarkerBasedLocalizerNode() {}
  /// Initialization
  void Init();

 private:
  /// Estimate self-position from marker recognition results
  void MarkerSubscriptionCallback_(const tmc_vision_msgs::msg::RecognizedObject::SharedPtr marker_msg);
  /// Obtain odometry and update the time stopped
  void OdometrySubscriptionCallback_(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void JointStateSubscriptionCallback_(const sensor_msgs::msg::JointState::SharedPtr joint_state);
  void StartServiceCallback_(std_srvs::srv::Empty::Request::SharedPtr req,
                             std_srvs::srv::Empty::Response::SharedPtr res);
  void StopServiceCallback_(std_srvs::srv::Empty::Request::SharedPtr req,
                            std_srvs::srv::Empty::Response::SharedPtr res);
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<tmc_vision_msgs::msg::RecognizedObject>::SharedPtr marker_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localized_pose_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_server_;
  std::string floor_frame_name_;
  std::string base_frame_name_;
  sensor_msgs::msg::JointState joint_state_;
  std::vector<std::string> joints_list_;
  /// Total movement amount since the last marker self-position correction. Unit: m
  double travel_distance_;
  /// External parameter. Enable marker self-position correction when the movement amount reaches this value
  double travel_distance_threshold_;
  /// Correct self-position if the distance to the marker is less than this distance [m]
  double marker_to_base_distance_threshold_;
  /// Flag indicating whether correction has been done even once
  bool is_first_localization_;
  /// Previous x-coordinate value. Used for calculating odometry movement amount.
  double pre_odom_x_;
  /// Previous y-coordinate value. Used for calculating odometry movement amount.
  double pre_odom_y_;
  /// Flag to enable marker self-position correction. Switched by service.
  bool enable_localization_;
  /// Joint speed considered as stopped
  double joint_stopping_vel_;
  /// tf transformation timeout time [s]
  double tf_time_out_;
  /// Vector managing object IDs and Poses of multiple Marker information
  std::vector<std::pair<uint32_t, geometry_msgs::msg::Pose> > marker_objects_;
};
}  // namespace tmc_marker_based_localizer

#endif  // TMC_MARKER_BASED_LOCALIZER_MARKER_BASED_LOCALIZER_HPP_
