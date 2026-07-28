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
#include <map>

#include <rclcpp/wait_for_message.hpp>

#include "marker_based_localizer.hpp"
#include "param.hpp"

namespace {
/// Enable marker recognition self-position correction if moved beyond this distance [m]
const double kDefaultTravelDistanceThreshold = 5.0;
/// Perform self-position correction if the distance to the marker is less than this distance [m]
const double kDefaultMarkerToBaseDistanceThreshold = 1.5;
const char* kMarkerTopicName = "recognized_object";
const char* kOdometryTopicName = "odom";
const char* kLocalizedPoseTopicName = "laser_2d_correct_pose";
const char* kJointStateTopicName = "joint_states";
const double kDefaultJointStoppingVel = 0.001;
const char* kDefaultBaseTfName = "base_footprint";
const char* kDefaultMapFrameName = "map";
const char* kParamNameMarkerObjects = "marker_objects";
const char* kParamNameObjectId = "object_id";
const double kTfTimeOut = 5.0;
const double kJointStateTimeOut = 60.0;
/// Log message issuance cycle [s]
const double kConsoleMessageIndicatePeriod = 1.0;
/// Buffer size for the topic
const uint32_t kTopicBufferSize = 1;

bool IsAllJointsStop(const std::vector<double>& velocity, double threshold) {
  for (std::vector<double>::const_iterator it = velocity.begin(); it != velocity.end(); ++it) {
    if (fabs(*it) > threshold) {
      return false;
    }
  }
  return true;
}
}  // unnamed namespace

namespace tmc_marker_based_localizer {
using std::placeholders::_1;
using std::placeholders::_2;

MarkerBasedLocalizerNode::MarkerBasedLocalizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("marker_based_localizer", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      floor_frame_name_(kDefaultMapFrameName),
      base_frame_name_(""),
      travel_distance_(0.0),
      travel_distance_threshold_(0.0),
      marker_to_base_distance_threshold_(0.0),
      is_first_localization_(true),
      pre_odom_x_(0.0),
      pre_odom_y_(0.0),
      enable_localization_(true),
      joint_stopping_vel_(0.0) {}

// Initialization
void MarkerBasedLocalizerNode::Init() {
  // Parameter acquisition
  GetOptionalParam(shared_from_this(), "base_tf_name", base_frame_name_, std::string(kDefaultBaseTfName));
  GetOptionalParam(shared_from_this(), "travel_distance_threshold", travel_distance_threshold_,
                   kDefaultTravelDistanceThreshold);
  GetOptionalParam(shared_from_this(), "marker_to_base_distance_threshold", marker_to_base_distance_threshold_,
                   kDefaultMarkerToBaseDistanceThreshold);
  GetParam(shared_from_this(), "joints_list", joints_list_);
  GetOptionalParam(shared_from_this(), "joint_stopping_vel", joint_stopping_vel_, kDefaultJointStoppingVel);

  std::map<std::string, rclcpp::Parameter> marker_objects;
  GetGroupParam(shared_from_this(), kParamNameMarkerObjects, marker_objects);
  std::vector<std::string> object_list;
  for (auto it = marker_objects.begin(); it != marker_objects.end(); ++it) {
    const int32_t sbstr_index = it->first.find(".");
    if (sbstr_index == static_cast<int32_t>(std::string::npos)) {
      throw std::runtime_error("object_id,translation,rotation parameter was not found.");
    }
    const std::string marker_object_name = it->first.substr(0, sbstr_index);
    if (std::find(object_list.begin(), object_list.end(), marker_object_name) == object_list.end()) {
      object_list.push_back(marker_object_name);
      std::map<std::string, rclcpp::Parameter> marker_object_param;
      GetGroupParam(marker_objects, marker_object_name, marker_object_param);
      int32_t object_id;
      if (!GetParam(marker_object_param, kParamNameObjectId, object_id)) {
        throw std::runtime_error("object_id parameter was not found.");
      }
      std::vector<double> translation(3);
      if (!GetParam(marker_object_param, "translation", translation)) {
        throw std::runtime_error("translation parameter was not found.");
      }
      std::vector<double> rotation(4);
      if (!GetParam(marker_object_param, "rotation", rotation)) {
        throw std::runtime_error("rotation parameter was not found.");
      }
      geometry_msgs::msg::Pose object_pose;
      object_pose.position.x = translation[0];
      object_pose.position.y = translation[1];
      object_pose.position.z = translation[2];
      object_pose.orientation.x = rotation[0];
      object_pose.orientation.y = rotation[1];
      object_pose.orientation.z = rotation[2];
      object_pose.orientation.w = rotation[3];

      std::pair<int32_t, geometry_msgs::msg::Pose> marker_object = std::make_pair(object_id, object_pose);
      marker_objects_.push_back(marker_object);
    }
  }

  // Subscriber settings
  marker_subscriber_ = this->create_subscription<tmc_vision_msgs::msg::RecognizedObject>(
      kMarkerTopicName, kTopicBufferSize,
      std::bind(&MarkerBasedLocalizerNode::MarkerSubscriptionCallback_, this, _1));
  odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      kOdometryTopicName, kTopicBufferSize,
      std::bind(&MarkerBasedLocalizerNode::OdometrySubscriptionCallback_, this, _1));
  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      kJointStateTopicName, kTopicBufferSize,
      std::bind(&MarkerBasedLocalizerNode::JointStateSubscriptionCallback_, this, _1));

  // Publisher settings
  localized_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      kLocalizedPoseTopicName, kTopicBufferSize);

  // Service settings
  start_service_server_ = this->create_service<std_srvs::srv::Empty>(
      "~/start_marker_based_localizer", std::bind(&MarkerBasedLocalizerNode::StartServiceCallback_, this, _1, _2));
  stop_service_server_ = this->create_service<std_srvs::srv::Empty>(
      "~/stop_marker_based_localizer", std::bind(&MarkerBasedLocalizerNode::StopServiceCallback_, this, _1, _2));

  sensor_msgs::msg::JointState joint_state;
  bool is_success = rclcpp::wait_for_message(
      joint_state, shared_from_this(), kJointStateTopicName,
      std::chrono::milliseconds(static_cast<int32_t>(kJointStateTimeOut * 1000)));
  if (is_success) {
    joint_state_ = joint_state;
  } else {
    std::stringstream error_message;
    error_message << joint_state_subscriber_->get_topic_name() << " is not received.";
    throw std::runtime_error(error_message.str());
  }
}

void MarkerBasedLocalizerNode::MarkerSubscriptionCallback_(
    const tmc_vision_msgs::msg::RecognizedObject::SharedPtr marker_msg) {
  // Skip processing if the enable switch is not activated
  if (!enable_localization_) {
    RCLCPP_DEBUG(this->get_logger(), "This has been disabled with service call.");
    return;
  }

  if (joints_list_.size() > 0) {
    // Retrieve the speed of axes listed in joints_list
    std::vector<double> joints_velocity;
    for (std::vector<std::string>::iterator it = joints_list_.begin(); it != joints_list_.end(); ++it) {
      std::vector<std::string>::iterator joint_name_it =
          std::find(joint_state_.name.begin(), joint_state_.name.end(), *it);
      if (joint_name_it != joint_state_.name.end()) {
        int32_t index = std::distance(joint_state_.name.begin(), joint_name_it);
        joints_velocity.push_back(joint_state_.velocity.at(index));
      } else {
        auto clock = rclcpp::Clock(RCL_ROS_TIME);
        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), clock, kConsoleMessageIndicatePeriod * 1000,
                                     *it << " is not found in JointState msg");
        return;
      }
    }

    // Consider as stopped if the speed of all axes in joints_list is below the specified value
    if (!IsAllJointsStop(joints_velocity, joint_stopping_vel_)) {
      RCLCPP_DEBUG(this->get_logger(), "Some joints are moving");
      return;
    }
  } else {
    RCLCPP_WARN_ONCE(this->get_logger(), "joints_list is not set.");
  }

  // Check if the marker is subject to self-position correction
  int32_t object_index = -1;
  for (uint32_t i = 0; i < marker_objects_.size(); ++i) {
    if (marker_objects_[i].first == marker_msg->object_id.object_id) {
      object_index = i;
      break;
    }
  }
  if (object_index < 0) {
    RCLCPP_DEBUG(this->get_logger(), "Cannot found object_id : %d", marker_msg->object_id.object_id);
    return;
  }
  const geometry_msgs::msg::Pose object_pose = marker_objects_[object_index].second;

  // If not the first marker self-position correction, check the movement amount
  // Skip processing if the movement amount does not exceed the threshold
  if (!is_first_localization_) {
    // For subsequent corrections, determine whether to execute processing based on the movement threshold
    if (travel_distance_ < travel_distance_threshold_) {
      // Skip processing if the movement amount is below the threshold
      RCLCPP_DEBUG(this->get_logger(),
          "Travel distance(%.1fm) is less than the threshold(%.1fm)."
          "Marker information is not used.",
          travel_distance_, travel_distance_threshold_);
      return;
    }
  }

  // Convert from Pose to PoseStamped due to doTransform argument specifications
  geometry_msgs::msg::PoseStamped marker_pose_in_camera_frame;
  marker_pose_in_camera_frame.header = marker_msg->header;
  marker_pose_in_camera_frame.pose = marker_msg->object_frame;
  geometry_msgs::msg::PoseStamped marker_pose_in_base_frame;
  marker_pose_in_base_frame.header.stamp = joint_state_.header.stamp;
  marker_pose_in_base_frame.header.frame_id = base_frame_name_;

  if (tf_buffer_.canTransform(base_frame_name_, marker_pose_in_camera_frame.header.frame_id,
                              marker_pose_in_base_frame.header.stamp, rclcpp::Duration::from_seconds(kTfTimeOut))) {
      geometry_msgs::msg::TransformStamped basefootprint_to_marker_stamped;
      basefootprint_to_marker_stamped = tf_buffer_.lookupTransform(base_frame_name_,
                                                                   marker_pose_in_camera_frame.header.frame_id,
                                                                   marker_pose_in_base_frame.header.stamp);
      tf2::doTransform(marker_pose_in_camera_frame, marker_pose_in_base_frame, basefootprint_to_marker_stamped);

  } else {
    auto clock = rclcpp::Clock(RCL_ROS_TIME);
    RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, kConsoleMessageIndicatePeriod * 1000,
        "Cannot transform %s to %s", base_frame_name_.c_str(), marker_pose_in_camera_frame.header.frame_id.c_str());
    return;
  }
  // Obtain the simultaneous transformation matrix of the marker's pose
  Eigen::Affine3d base_to_marker;
  tmc_eigen_bridge::PoseMsgToAffine3d(marker_pose_in_base_frame.pose, base_to_marker);

  // Transform to the base pose relative to the marker frame
  Eigen::Affine3d marker_to_base = base_to_marker.inverse();

  double squared_distance_to_marker = marker_to_base.translation().x() * marker_to_base.translation().x() +
                                      marker_to_base.translation().y() * marker_to_base.translation().y();
  RCLCPP_DEBUG(this->get_logger(), "marker_to_base: %lf %lf", squared_distance_to_marker,
               marker_to_base_distance_threshold_ * marker_to_base_distance_threshold_);

  // Skip processing if the distance between the marker and the robot is too far
  if (squared_distance_to_marker > marker_to_base_distance_threshold_ * marker_to_base_distance_threshold_) {
    auto clock = rclcpp::Clock(RCL_ROS_TIME);
    RCLCPP_INFO_THROTTLE(this->get_logger(), clock, kConsoleMessageIndicatePeriod * 1000,
                         "Distance from marker to base is far");
    return;
  }

  // Obtain the pose of the object (marker) relative to the floor frame from object information
  Eigen::Affine3d floor_to_marker;
  tmc_eigen_bridge::PoseMsgToAffine3d(object_pose, floor_to_marker);

  // Calculate the robot frame relative to the floor frame from two simultaneous transformation matrices
  Eigen::Affine3d floor_to_base = floor_to_marker * marker_to_base;

  // Convert to a message for publishing and publish
  geometry_msgs::msg::PoseWithCovarianceStamped localized_pose;
  tmc_eigen_bridge::Affine3dToPoseMsg(floor_to_base, localized_pose.pose.pose);
  localized_pose.header.frame_id = floor_frame_name_;
  localized_pose.header.stamp = marker_msg->header.stamp;
  localized_pose_publisher_->publish(localized_pose);

  auto clock = rclcpp::Clock(RCL_ROS_TIME);
  RCLCPP_INFO_THROTTLE(this->get_logger(), clock, kConsoleMessageIndicatePeriod * 1000,
                       "Robot is localized using the marker information.");
  // Reset the movement threshold after correction is completed
  travel_distance_ = 0.0;
  // Do not lower the initial flag until correction is completed
  is_first_localization_ = false;
}

void MarkerBasedLocalizerNode::OdometrySubscriptionCallback_(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  // Accumulate movement amount
  travel_distance_ +=
      sqrt((odom_msg->pose.pose.position.x - pre_odom_x_) * (odom_msg->pose.pose.position.x - pre_odom_x_) +
           (odom_msg->pose.pose.position.y - pre_odom_y_) * (odom_msg->pose.pose.position.y - pre_odom_y_));
  pre_odom_x_ = odom_msg->pose.pose.position.x;
  pre_odom_y_ = odom_msg->pose.pose.position.y;
}

void MarkerBasedLocalizerNode::JointStateSubscriptionCallback_(
    const sensor_msgs::msg::JointState::SharedPtr joint_state) {
  joint_state_ = *joint_state;
}

// Service to start functionality
void MarkerBasedLocalizerNode::StartServiceCallback_(
    std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res) {
  enable_localization_ = true;
}

// Service to stop functionality
void MarkerBasedLocalizerNode::StopServiceCallback_(
    std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res) {
  enable_localization_ = false;
}
}  // namespace tmc_marker_based_localizer
