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
/// @file viewpoint_controller.cpp
/// @brief Node to control the viewpoint using the neck pan axis
#include <tmc_viewpoint_controller/viewpoint_controller_node.hpp>
#include <algorithm>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <angles/angles.h>

#include <tmc_viewpoint_controller/param.hpp>
// TODO(syuuhei_shiro): filter_trajectory_with_constraintsサービスがROS2化されたら使用するよう対応する
// #include <tmc_manipulation_msgs/FilterJointTrajectoryWithConstraints.h>

namespace {
/// Joint axis topic name
const char* const kDefaultJointStatesTopicName = "joint_states";
/// Neck trajectory topic name
const char* const kDefaultCommandTopicName = "command";
/// Service name to turn the function on
const char* const kStartServiceName = "~/start";
/// Service name to turn the function off
const char* const kStopServiceName = "~/stop";
/// Service name to switch to Tracking mode
const char* const kTrackingModeServiceName = "~/set_viewpoint_mode_tracking";
/// Service name to switch to Path mode
const char* const kPathModeServiceName = "~/set_viewpoint_mode_path";
/// tf map frame name
const char* const kDefaultMapFrameName = "map";
/// tf base frame name
const char* const kDefaultBaseFrameName = "base_footprint";
/// Neck pan axis name
const char* const kDefaultNeckPanName = "head_pan_joint";
/// Neck tilt axis name
const char* const kDefaultNeckTiltName = "head_tilt_joint";
/// Default neck tilt angle [rad]
const double kNeckTiltDefaultAngle = 0.0;
/// Drive cycle [Hz]
const double kDefaultRate = 1.0;
/// Maximum neck pan axis rotation per cycle [rad]
const double kDefaultMaxRotationOnce = 0.6;
/// Neck right pan axis mechanical limit [rad]
const double kDefaultHeadPanMin = -210 * M_PI / 180;
/// Neck left pan axis mechanical limit [rad]
const double kDefaultHeadPanMax = 110 * M_PI / 180;
/// Service establishment preparation allowable time [sec]
const double kTimeout = 120.0;
/// Log message issuance cycle [ms]
const int32_t kConsoleMessageIndicatePeriod = 5000;
/// Viewpoint control mode
enum ViewpointControlMode {
  /// Mode to direct the viewpoint towards the path direction
  kModePath = 0,
  /// Mode to direct the viewpoint towards the target direction
  kModeTrackingTarget = 1
};


/// Clamp the input value within the range of minval to maxval
double Clamp(const double x, const double minval, const double maxval) {
  assert(minval < maxval && "min,maxval is wrong range!");
  if (x < minval) return minval;
  if (x > maxval) return maxval;
  return x;
}

// TODO(syuuhei_shiro): filter_trajectory_with_constraintsサービスがROS2化されたら使用するよう対応する
#if 0
/// Generate trajectory with trajectory_filter
bool FilterTrajectory(const trajectory_msgs::JointTrajectory& trajectory, trajectory_msgs::JointTrajectory& filtered) {
  // Apply filter (trajectory_filter)
  tmc_manipulation_msgs::FilterJointTrajectoryWithConstraints filter;
  filter.request.trajectory = trajectory;
  filter.request.allowed_time = ros::Duration(10);
  if (!ros::service::call("trajectory_filter/filter_trajectory_with_constraints", filter)) {
    ROS_WARN("Fail to call trajectory_filter");
    return false;
  }
  if (filter.response.error_code.val != filter.response.error_code.SUCCESS) {
    ROS_ERROR("Fail to filter trajectory");
    return false;
  }
  std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator front;
  front = filter.response.trajectory.points.begin();
  filter.response.trajectory.points.erase(front);
  filtered = filter.response.trajectory;
  return true;
}
#else
/// As a temporary measure until the filter_trajectory_with_constraints service is ported to ROS2, generate the trajectory independently
bool FilterTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
    trajectory_msgs::msg::JointTrajectory& filtered) {
  filtered.joint_names = trajectory.joint_names;
  filtered.points.resize(1);
  filtered.points[0].positions.resize(2);
  filtered.points[0].positions[0] = trajectory.points[1].positions[0];
  filtered.points[0].positions[1] = trajectory.points[1].positions[1];
  filtered.points[0].velocities.resize(2);
  filtered.points[0].velocities[0] = 0.0;
  filtered.points[0].velocities[1] = 0.0;
  filtered.points[0].accelerations.resize(2);
  filtered.points[0].accelerations[0] = 0.0;
  filtered.points[0].accelerations[1] = 0.0;
  // Reflect the size of the trajectory to be followed to some extent in time_from_start
  // Originally, the trajectory was generated with a speed limit of 1 rad/s and an acceleration limit of 1 rad/s^2, so roughly set the limit to half the speed
  constexpr double kTemporalMaxRotationSpeed = 0.5;
  const double head_pan_diff = fabs(trajectory.points[1].positions[0] - trajectory.points[0].positions[0]);
  const double head_pan_time = head_pan_diff / kTemporalMaxRotationSpeed;
  const double head_tilt_diff = fabs(trajectory.points[1].positions[1] - trajectory.points[0].positions[1]);
  const double head_tilt_time = head_tilt_diff / kTemporalMaxRotationSpeed;
  const double time_from_start = std::max(head_pan_time, head_tilt_time);
  filtered.points[0].time_from_start = rclcpp::Duration::from_seconds(time_from_start);
  return true;
}
#endif
}  // end anonymous namespace

namespace tmc_viewpoint_controller {
using std::placeholders::_1;
using std::placeholders::_2;
/// Constructor
ViewpointControllerNode::ViewpointControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("viewpoint_controller", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      current_neck_pan_angle_(0.0),
      current_neck_tilt_angle_(0.0),
      enable_view_ctrl_(true),
      max_rotation_once_(0.0),
      head_pan_min_(0.0),
      head_pan_max_(0.0),
      fixed_neck_tilt_(0.0),
      rate_(0.0),
      viewpoint_control_mode_(kModePath) {}

void ViewpointControllerNode::Init() {
  viewpoint_to_path_ = std::make_shared<ViewpointToPath>(shared_from_this());
  viewpoint_to_tracking_target_ = std::make_shared<ViewpointToTrackingTarget>(shared_from_this());

  std::string robot_pose_topic;
  std::string command_topic;
  std::string joint_states_topic;

  // Parameter acquisition process
  // tf frame name
  GetOptionalParam(shared_from_this(), "map_frame", map_frame_, std::string(kDefaultMapFrameName));
  GetOptionalParam(shared_from_this(), "base_frame", base_frame_, std::string(kDefaultBaseFrameName));
  // Topic name
  GetOptionalParam(shared_from_this(), "joint_states", joint_states_topic, std::string(kDefaultJointStatesTopicName));
  GetOptionalParam(shared_from_this(), "command", command_topic, std::string(kDefaultCommandTopicName));
  // Maximum neck rotation per cycle
  GetOptionalParam(shared_from_this(), "max_rotation_once", max_rotation_once_, kDefaultMaxRotationOnce);
  // Neck right rotation mechanical limit
  GetOptionalParam(shared_from_this(), "head_pan_min", head_pan_min_, kDefaultHeadPanMin);
  // Neck left rotation mechanical limit
  GetOptionalParam(shared_from_this(), "head_pan_max", head_pan_max_, kDefaultHeadPanMax);
  // Fixed tilt axis angle
  GetOptionalParam(shared_from_this(), "fixed_neck_tilt_angle", fixed_neck_tilt_, kNeckTiltDefaultAngle);
  // Drive cycle
  GetOptionalParam(shared_from_this(), "rate", rate_, kDefaultRate);
  // Get axis name
  GetOptionalParam(shared_from_this(), "neck_pan_name", neck_pan_name_, std::string(kDefaultNeckPanName));
  GetOptionalParam(shared_from_this(), "neck_tilt_name", neck_tilt_name_, std::string(kDefaultNeckTiltName));

  // TODO(syuuhei_shiro): filter_trajectory_with_constraintsサービスがROS2化されたら使用するよう対応する
#if 0
  if (!ros::service::waitForService("trajectory_filter/filter_trajectory_with_constraints", ros::Duration(kTimeout))) {
    throw std::runtime_error("trajectory_filter service is not exist");
  }
#endif

  // JointStates Subscriber setup
  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, 1, std::bind(&ViewpointControllerNode::CallbackJointState, this, _1));

  // Neck trajectory Publisher setup
  command_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(command_topic, 1);

  // Define function On/Off services
  start_service_ = this->create_service<std_srvs::srv::Empty>(
      kStartServiceName, std::bind(&ViewpointControllerNode::StartServiceCallback, this, _1, _2));
  stop_service_ = this->create_service<std_srvs::srv::Empty>(
      kStopServiceName, std::bind(&ViewpointControllerNode::StopServiceCallback, this, _1, _2));

  // Define viewpoint mode (tracking) switch service
  set_viewpoint_mode_tracking_service_ = this->create_service<std_srvs::srv::Empty>(kTrackingModeServiceName,
      std::bind(&ViewpointControllerNode::SetViewpointModeTrackingServiceCallback, this, _1, _2));
  // Define viewpoint mode (path) switch service
  set_viewpoint_mode_path_service_ = this->create_service<std_srvs::srv::Empty>(
      kPathModeServiceName, std::bind(&ViewpointControllerNode::SetViewpointModePathServiceCallback, this, _1, _2));
}

/// Destructor
ViewpointControllerNode::~ViewpointControllerNode() {}

/// Modify command values considering neck axis limits and current neck angle
double ViewpointControllerNode::NeckPanningFilter(const double command) {
  double tmp_out;
  double output;
  // Modify command values considering the limit of rotation per cycle
  tmp_out = Clamp(command, current_neck_pan_angle_ - max_rotation_once_, current_neck_pan_angle_ + max_rotation_once_);
  // Consider mechanical limits
  output = Clamp(tmp_out, head_pan_min_, head_pan_max_);
  return output;
}

/// Change viewpoint
void ViewpointControllerNode::ChangeViewpoint() {
  tf2::Stamped<tf2::Transform> map_to_robot;
  bool is_detect_direction = false;
  double robot_view_direction = 0.0;
  try {
    if (!tf_buffer_.canTransform(map_frame_, base_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(5.0))) {
      throw std::runtime_error("waitTransform Error map_frame to base_frame");
    }
    geometry_msgs::msg::TransformStamped map_to_robot_stamped;
    map_to_robot_stamped = tf_buffer_.lookupTransform(map_frame_, base_frame_, rclcpp::Time(0));
    tf2::fromMsg(map_to_robot_stamped, map_to_robot);
    // Robot self-position
    Eigen::Vector3d robot_pose;
    robot_pose << map_to_robot.getOrigin().getX(), map_to_robot.getOrigin().getY(),
        tf2::getYaw(map_to_robot.getRotation());

    if (viewpoint_control_mode_ == kModeTrackingTarget) {
      // Tracking mode
      is_detect_direction = viewpoint_to_tracking_target_->ViewpointToTrackingTargetDircetion(
          robot_pose, robot_view_direction);
    } else {
      // Path mode
      is_detect_direction = viewpoint_to_path_->ViewPointToPathDirection(robot_pose, robot_view_direction);
    }
  } catch (const std::exception& e) {
    auto clock = rclcpp::Clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(this->get_logger(), clock, kConsoleMessageIndicatePeriod, "%s", e.what());
  }

  // Request command_trajectory once viewpoint planning is completed
  if (is_detect_direction) {
    double filtered_view_direction = NeckPanningFilter(robot_view_direction);
    // Generate command_trajectory
    trajectory_msgs::msg::JointTrajectory command_trajectory;
    command_trajectory.joint_names.push_back(neck_pan_name_);
    command_trajectory.joint_names.push_back(neck_tilt_name_);
    command_trajectory.points.resize(2);
    command_trajectory.points[0].positions.resize(2);
    command_trajectory.points[0].positions[0] = current_neck_pan_angle_;
    command_trajectory.points[0].positions[1] = current_neck_tilt_angle_;
    command_trajectory.points[1].positions.resize(2);
    command_trajectory.points[1].positions[0] = filtered_view_direction;
    command_trajectory.points[1].positions[1] = fixed_neck_tilt_;
    trajectory_msgs::msg::JointTrajectory filtered_trajectory;
    // Generate trajectory with trajectory_filter
    if (!FilterTrajectory(command_trajectory, filtered_trajectory)) {
      RCLCPP_ERROR(this->get_logger(), "Fail to call FilterTrajectory");
      return;
    }
    command_trajectory_pub_->publish(filtered_trajectory);
  }
}

/// JointStates acquisition callback function
void ViewpointControllerNode::CallbackJointState(const sensor_msgs::msg::JointState::SharedPtr joint_states) {
  // Find neck pan axis from joint_states
  std::vector<std::string>::iterator pan_name_it =
      std::find(joint_states->name.begin(), joint_states->name.end(), neck_pan_name_);
  if (pan_name_it != joint_states->name.end()) {
    int32_t index = std::distance(joint_states->name.begin(), pan_name_it);
    current_neck_pan_angle_ = joint_states->position[index];
  } else {
    throw std::runtime_error("There is no NECK_PAN joint");
  }
  // Find neck tilt axis from joint_states
  std::vector<std::string>::iterator tilt_name_it =
      std::find(joint_states->name.begin(), joint_states->name.end(), neck_tilt_name_);
  if (tilt_name_it != joint_states->name.end()) {
    int32_t index = std::distance(joint_states->name.begin(), tilt_name_it);
    current_neck_tilt_angle_ = joint_states->position[index];
  } else {
    throw std::runtime_error("There is no NECK_TILT joint");
  }
}

/// Viewpoint control function On service
void ViewpointControllerNode::StartServiceCallback(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res) {
  enable_view_ctrl_ = true;
  RCLCPP_INFO(rclcpp::get_logger("view_point_controller"), "view_ctrl enable");
}

/// Viewpoint control function Off service
void ViewpointControllerNode::StopServiceCallback(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res) {
  enable_view_ctrl_ = false;
  RCLCPP_INFO(rclcpp::get_logger("view_point_controller"), "view_ctrl disable");
}
/// Set control mode to Path (direct viewpoint towards own path)
void ViewpointControllerNode::SetViewpointModePathServiceCallback(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res) {
  RCLCPP_INFO(rclcpp::get_logger("view_point_controller"), "view_ctrl path mode");
  viewpoint_control_mode_ = kModePath;
}

/// Set control mode to Tracking (direct viewpoint towards target)
void ViewpointControllerNode::SetViewpointModeTrackingServiceCallback(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res) {
  RCLCPP_INFO(rclcpp::get_logger("view_point_controller"), "view_ctrl tracking mode");
  viewpoint_control_mode_ = kModeTrackingTarget;
}

/// Ros Spin
void ViewpointControllerNode::Run() {
  rclcpp::Rate rate(rate_);
  while (rclcpp::ok()) {
    rclcpp::spin_some(shared_from_this());
    // Viewpoint control conditions
    if (enable_view_ctrl_) {
      ChangeViewpoint();
    }
    rate.sleep();
  }
}
}  // end namespace tmc_viewpoint_controller
