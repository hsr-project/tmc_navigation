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
/// @file     pose_integrator_node.cpp
/// @brief    Integrate multiple self-positioning results (implementation part)
/// @version  0.2.0
/// @author   Takao Yasuda
/// @author   Applied for Partner-Robot Coding Rule(Ver:x.xx)
/// @date     2012.05.08
#include "pose_integrator_node.hpp"
#include <chrono>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/// Anonymous namespace (pose_integrator)
namespace {
/// Default operation cycle (sec): Same as the global_pose publishing cycle
double const kDefaultCycleTime = 0.005;
/// default cycletime of tf broadcast(sec)
double const kDefaultTfCycleTime = 1.0/30.0;
/// Topic buffer size
uint32_t const kTopicBufferSize = 10;
/// Node name
const char* const kNodeName = "pose_integrator";
/// Default frame name of the global coordinate system
const char* const kGlobalFrameId = "map";
/// Default frame name of the robot cart
const char* const kDefaultBaseFrameId = "base_footprint";
const char* const kDefaultOdomFrameId = "odom";
/// Topic name (self-positioning)
const char* const kGlobalPoseTopicName = "global_pose";
/// Topic name (odometry)
const char* const kOdometryTopicName = "odometry";
/// Topic name (laser self-positioning)
const char* const kLaserPoseTopicName = "laser_2d_pose";
/// Parameter name (cycle)
const char* const kParameterNameCycleTime = "cycle_time";
/// Parameter name (tf broadcast cycle time)
const char* const kParameterNameTfCycleTime = "tf_cycle_time";
/// Parameter name (convergence time)
const char* const kParameterNameConvergenceTime = "convergence_time";
/// Parameter name (rear URG)
const char* const kParameterNameInvertedUrg = "inverted_urg";
/// Parameter name (cart's tf name)
const char* const kParameterNameRobotTfName = "robot_tf_name";
/// Parameter name for translational speed considered as the cart being stopped
const char* const kParameterNameStopTranslationalVel = "stop_translational_vel_threshold";
/// Parameter name for in-place rotational speed considered as the cart being stopped
const char* const kParameterNameStopRotationalVel = "stop_rotational_vel_threshold";
/// Default value of translational speed considered as the cart being stopped [m/s]
double const kDefalutStopTranslationalVel = 0.001;
/// Default value of in-place rotational speed considered as the cart being stopped [rad/s]
double const kDefalutStopRotationalVel = 0.001;
/// Warning cycle [ms] when TF cannot be read
int32_t const kWarnLogIndicatePeriod = 10000;

// Retrieve required parameters
template<typename T>
bool GetParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value) {
  rclcpp::Parameter param;
  if (!node->get_parameter(param_name, param)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("pose_integrator"),
        "Parameter '" << param_name << "' is not specified.");
    return false;
  }
  value = param.get_value<T>();
  return true;
}

// Retrieve optional parameters
template<typename T>
void GetOptionalParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value,
                      const T& default_value) {
  if (!GetParam(node, param_name, value)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("pose_integrator"),
        "Used default value: " << default_value);
    value = default_value;
  }
}
}  // namespace

/// Namespace (tmc_pose_integrator)
namespace tmc_pose_integrator {
using std::chrono::milliseconds;
using std::placeholders::_1;

PoseIntegratorNode::PoseIntegratorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("pose_integrator", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      tf_broadcaster_(*this),
      is_first_laser_2d_pose_(true),
      is_inverted_urg_(false),
      is_odom_updated_(false),
      robot_base_tf_yaw_(0.0),
      pre_laser_2d_pose_time_(0.0),
      global_frame_id_(kGlobalFrameId),
      pose_integrator_(PoseIntegrator::Ptr(new PoseIntegrator())),
      latest_global_pose_tf_stamp_(0.0) {}

// Initialization
void PoseIntegratorNode::Init() {
  global_pose_.x = 0.0;
  global_pose_.y = 0.0;
  global_pose_.theta = 0.0;

  // Retrieve operation cycle parameter from the parameter server
  double cycle_time = 0.0;
  if (!GetParam(shared_from_this(), kParameterNameCycleTime, cycle_time)) {
    std::string msg = "Not set ";
    msg += kParameterNameCycleTime;
    throw std::runtime_error(msg);
  }
  if (!(cycle_time > std::numeric_limits<double>::epsilon())) {
    RCLCPP_WARN(this->get_logger(), "Cycle time is too small. Default value (%f) is used.", kDefaultCycleTime);
    cycle_time = kDefaultCycleTime;
  }

  // Set the cycle for the self-position integration object.
  pose_integrator_->set_cycle_time(cycle_time);
  rate_ = std::make_shared<rclcpp::Rate>(1.0 / cycle_time);


  // tf broadcast cycletime
  double tf_cycle_time = 0.0;
  if (!GetParam(shared_from_this(), kParameterNameTfCycleTime, tf_cycle_time)) {
    std::string msg = "Not set ";
    msg += kParameterNameTfCycleTime;
    throw std::runtime_error(msg);
  }
  if (!(tf_cycle_time > std::numeric_limits<double>::epsilon())) {
    RCLCPP_WARN(this->get_logger(), "Tf cycle time is too small. Default value (%f) is used.", kDefaultTfCycleTime);
    tf_cycle_time = kDefaultTfCycleTime;
  }

  // Retrieve convergence time parameter
  double convergence_time = 0.0;
  if (!GetParam(shared_from_this(), kParameterNameConvergenceTime, convergence_time)) {
    std::string msg = "Not set ";
    msg += kParameterNameConvergenceTime;
    throw std::runtime_error(msg);
  }
  pose_integrator_->set_convergence_time(convergence_time);

  // Rear URG
  if (!GetParam(shared_from_this(), kParameterNameInvertedUrg, is_inverted_urg_)) {
    std::string msg = "Not set ";
    msg += kParameterNameInvertedUrg;
    throw std::runtime_error(msg);
  }
  // Threshold for determining stop (translational speed)
  double value;
  GetOptionalParam(shared_from_this(), kParameterNameStopTranslationalVel, value, kDefalutStopTranslationalVel);
  pose_integrator_->set_stop_translational_vel(value);
  // Threshold for determining stop (in-place rotational speed)
  GetOptionalParam(shared_from_this(), kParameterNameStopRotationalVel, value, kDefalutStopRotationalVel);
  pose_integrator_->set_stop_rotational_vel(value);

  GetOptionalParam(shared_from_this(), "odom_tf_name", odom_frame_id_, std::string(kDefaultOdomFrameId));
  GetOptionalParam(shared_from_this(), "base_tf_name", base_frame_id_, std::string(kDefaultBaseFrameId));

  // Subscriber setup: Self-position estimation using 2D laser data
  subscribe_laser2d_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      kLaserPoseTopicName, kTopicBufferSize, std::bind(&PoseIntegratorNode::CallbackLaser2dPose_, this, _1));
  // Publisher setup: Self-position estimation
  global_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      kGlobalPoseTopicName, kTopicBufferSize);
  // initialize timer for tf broadcast
  timer_ = this->create_wall_timer(milliseconds(static_cast<int32_t>(tf_cycle_time * 1000)),
      std::bind(&PoseIntegratorNode::CallbackTfBroadcast_, this));
}

/// Since the object is created on the stack, memory is not explicitly released.
PoseIntegratorNode::~PoseIntegratorNode() {
  pose_integrator_.reset();
}

/// @param[in] laser_2d_msg 2D LRF self-position message
void PoseIntegratorNode::CallbackLaser2dPose_(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr laser_2d_msg) {

  // Initialize a variable for checking time updates only on the first run
  if (is_first_laser_2d_pose_) {
    pre_laser_2d_pose_time_ = rclcpp::Time(laser_2d_msg->header.stamp).seconds();
    is_first_laser_2d_pose_ = false;
    return;
  }

  // Confirm that the timestamp of the self-position is updated
  // Since pose_integrator_ is not operating,
  // Discard data with a warning if it has not been updated
  if (!(fabs(pre_laser_2d_pose_time_ - rclcpp::Time(laser_2d_msg->header.stamp).seconds()) >
      std::numeric_limits<double>::epsilon())) {
    auto clock = rclcpp::Clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(this->get_logger(), clock, kWarnLogIndicatePeriod,
                         "Time stamp of laser_2d_pose is not changed.");
  }
  pre_laser_2d_pose_time_ = rclcpp::Time(laser_2d_msg->header.stamp).seconds();
  Pose2dWithCovariance laser_2d;
  laser_2d.x = laser_2d_msg->pose.pose.position.x;
  laser_2d.y = laser_2d_msg->pose.pose.position.y;
  laser_2d.theta = tf2::getYaw(laser_2d_msg->pose.pose.orientation);
  for (uint32_t i = 0; i < kCovarianceMatrixSize36; i++) {
    laser_2d.covariance[i] = laser_2d_msg->pose.covariance[i];
  }
  laser_2d.time = rclcpp::Time(laser_2d_msg->header.stamp).seconds();
  pose_integrator_->set_localized_2d_pose(laser_2d);

  // Retrieve global_pose from tf that matches the timestamp of laser_2d_pose
  tf2::Stamped<tf2::Transform> map_to_odom;
  Pose2d odom_synchronized_with_localizer;
  // Check if global_pose is being published && if the cart is moving
  if (rclcpp::Time(laser_2d_msg->header.stamp).seconds() < latest_global_pose_tf_stamp_ &&
      pose_integrator_->IsBaseMoving()) {
    try {
      if (!tf_buffer_.canTransform(global_frame_id_, base_frame_id_, laser_2d_msg->header.stamp,
                                    rclcpp::Duration::from_seconds(0.1))) {
        throw std::runtime_error("waitForTransform Error to get global_pose");
      }
      geometry_msgs::msg::TransformStamped map_to_odom_stamped = tf_buffer_.lookupTransform(
          global_frame_id_, base_frame_id_, laser_2d_msg->header.stamp);
      tf2::fromMsg(map_to_odom_stamped, map_to_odom);
    } catch (const std::exception& ex) {
      auto clock = rclcpp::Clock(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(this->get_logger(), clock, kWarnLogIndicatePeriod,
          "Couldn't transform \'%s\' to \'%s\': %s",
          base_frame_id_.c_str(), global_frame_id_.c_str(), ex.what());
      return;
    }
    odom_synchronized_with_localizer.x = map_to_odom.getOrigin().getX();
    odom_synchronized_with_localizer.y = map_to_odom.getOrigin().getY();
    odom_synchronized_with_localizer.theta = tf2::getYaw(map_to_odom.getRotation());

  } else {
    // If a global_pose synchronized with the timestamp of laser_2d_pose has not yet been published
    // Or if the cart is stopped, do not synchronize with the timestamp of laser_2d_pose
    // Use the latest global_pose
    odom_synchronized_with_localizer.x = global_pose_.x;
    odom_synchronized_with_localizer.y = global_pose_.y;
    odom_synchronized_with_localizer.theta = global_pose_.theta;
  }
  // Set the synchronized odometry
  pose_integrator_->set_synchronized_odometry(odom_synchronized_with_localizer);
}

/// timer callback for broadcasting tf
void PoseIntegratorNode::CallbackTfBroadcast_() {
  if (is_odom_updated_) {
    tf2::Transform odom_to_map_tf(tf2::Quaternion(odom_to_map_.getRotation()), tf2::Vector3(odom_to_map_.getOrigin()));
    tf2::Transform map_to_odom_tf = odom_to_map_tf.inverse();
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = tf2_ros::toMsg(odom_to_map_.stamp_);
    transform_stamped.header.frame_id = global_frame_id_.c_str();
    transform_stamped.child_frame_id = odom_frame_id_.c_str();

    transform_stamped.transform.translation.x = map_to_odom_tf.getOrigin().getX();
    transform_stamped.transform.translation.y = map_to_odom_tf.getOrigin().getY();
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = map_to_odom_tf.getRotation().getX();
    transform_stamped.transform.rotation.y = map_to_odom_tf.getRotation().getY();
    transform_stamped.transform.rotation.z = map_to_odom_tf.getRotation().getZ();
    transform_stamped.transform.rotation.w = map_to_odom_tf.getRotation().getW();
    tf_broadcaster_.sendTransform(transform_stamped);
    // Record the timestamp of the broadcast
    latest_global_pose_tf_stamp_ = tf2::timeToSec(odom_to_map_.stamp_);
    is_odom_updated_ = false;
  }
}

/// Retrieve odom from tf
void PoseIntegratorNode::GetOdometryFromTf(void) {
  tf2::Stamped<tf2::Transform> map_to_odom;
  try {
    if (!tf_buffer_.canTransform(odom_frame_id_, base_frame_id_, rclcpp::Time(0),
                                 rclcpp::Duration::from_seconds(0.1))) {
      throw std::runtime_error("waitForTransform Error to get Odometry");
    }
    geometry_msgs::msg::TransformStamped map_to_odom_stamped = tf_buffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, rclcpp::Time(0));
    tf2::fromMsg(map_to_odom_stamped, map_to_odom);
  } catch (const std::exception& e) {
    auto clock = rclcpp::Clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(this->get_logger(), clock, kWarnLogIndicatePeriod,
        "%s", e.what());
    return;
  }
  Pose2d odom;
  odom.x = map_to_odom.getOrigin().getX();
  odom.y = map_to_odom.getOrigin().getY();
  odom.theta = tf2::getYaw(map_to_odom.getRotation());
  pose_integrator_->set_odometry(odom);
}

/// Perform interpolation processing, and finally publish the result and update the tf frame.
/// If odometry has never been retrieved, do nothing.
void PoseIntegratorNode::UpdateGlobalPose() {
  rate_->sleep();
  // Retrieve odom from tf
  GetOdometryFromTf();

  // Do nothing until the first odometry value is received
  if (pose_integrator_->is_first_odometry_received()) {
    // Use time-synchronized odometry
    SendGlobalPose_(pose_integrator_->CorrectOdometryWithConvergenceAndSynchronization());
  } else {
    // Continue to issue warnings until odometry values are received
    auto clock = rclcpp::Clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(this->get_logger(), clock, kWarnLogIndicatePeriod,
                         "Odometry TF has never been set to pose_integrator. "
                         "Pose integrator could not publish integrated pose..");
  }
}

/// Publish the self-position integration result as a topic & update the tf frame
/// @param[in] pose Robot self-position
void PoseIntegratorNode::SendGlobalPose_(const Pose2d& pose) {
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = this->get_clock()->now();
  pose_stamped.header.frame_id = global_frame_id_;
  pose_stamped.pose.position.x = pose.x;
  pose_stamped.pose.position.y = pose.y;

  global_pose_ = pose;

  // When using URG for map generation, reverse the orientation of the robot
  double send_pose = 0.0;
  if (is_inverted_urg_) {
    send_pose = pose.theta + M_PI;
  } else {
    send_pose = pose.theta;
  }
  tf2::Quaternion q;
  q.setRPY(0, 0, send_pose);
  pose_stamped.pose.orientation = tf2::toMsg(q);
  // Publish the topic
  global_pose_publisher_->publish(pose_stamped);


  // subtracting robot to odom from map to robot and send map to odom instead
  // comply with ROS REP105
  // map_to_odom = odom_to_map.inverse
  // odom_to_map = odom_to_robot * robot_to_map
  try {
    tf2::Transform map_to_robot(
        tf2::Quaternion(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                       pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w),
        tf2::Vector3(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z));
    tf2::Stamped<tf2::Transform> robot_to_map(map_to_robot.inverse(), tf2_ros::fromMsg(pose_stamped.header.stamp),
        base_frame_id_);
    if (!tf_buffer_.canTransform(odom_frame_id_, base_frame_id_, rclcpp::Time(0),
        rclcpp::Duration::from_seconds(0.1))) {
      auto clock = rclcpp::Clock(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(this->get_logger(), clock, kWarnLogIndicatePeriod,
                           "waitForTransform Error to get odom_to_base. "
                           "Failed to broadcast map to odom transform.");
        return;
    }
    tf2::Stamped<tf2::Transform> trans;
    geometry_msgs::msg::TransformStamped trans_stamped = tf_buffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, rclcpp::Time(0));
    tf2::fromMsg(trans_stamped, trans);
    odom_to_map_.setData(trans * robot_to_map);
    odom_to_map_.frame_id_ = odom_frame_id_;
    odom_to_map_.stamp_ = tf2_ros::fromMsg(pose_stamped.header.stamp);
    is_odom_updated_ = true;
  } catch (tf2::TransformException) {
    auto clock = rclcpp::Clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(this->get_logger(), clock, kWarnLogIndicatePeriod,
        "Failed to subtract robot to odom transform");
    return;
  }
}
}  // namespace tmc_pose_integrator
