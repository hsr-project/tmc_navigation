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
#include "tmc_odometry_switcher/odometry_switcher.hpp"

#include <string>
#include <utility>

#include "tmc_odometry_switcher/param.hpp"

namespace {
const char* const kOdomInputListName = "odom_topics";
const char* const kInitialOdomName = "initial_odom";
const char* const kDefaultOdomName = "odom";
const char* const kDefaultOdomChildName = "base_footprint";
const char* const kOdomParamName = "odom_frame";
const char* const kOdomChildParamName = "odom_child_frame";
const char* const kPublishTopicName = "switched_odom";
const char* const kOdomSwitchServicetName = "odometry_switch";
}  // namespace


namespace tmc_odometry_switcher {
using std::placeholders::_1;
using std::placeholders::_2;

/// Conversion from nav_msgs::msg::Odometry to geometry_msgs::msg::TransformStamped
geometry_msgs::msg::TransformStamped OdometryMsgToTransform(const nav_msgs::msg::Odometry& input) {
  geometry_msgs::msg::TransformStamped output;
  output.header = input.header;
  output.child_frame_id = input.child_frame_id;
  output.transform.translation.x = input.pose.pose.position.x;
  output.transform.translation.y = input.pose.pose.position.y;
  output.transform.translation.z = input.pose.pose.position.z;
  output.transform.rotation = input.pose.pose.orientation;

  return output;
}

/// Constructor
OdometrySwitcherNode::OdometrySwitcherNode(const rclcpp::NodeOptions& options)
    : Node("odometry_switcher", options) {}

/// Initialization
void OdometrySwitcherNode::Init() {
  // Retrieve the list of odometry topics as source from parameters
  std::map<std::string, rclcpp::Parameter> odom_topic_list;
  if (!GetGroupParam(shared_from_this(), kOdomInputListName, odom_topic_list)) {
    throw std::runtime_error(std::string(kOdomInputListName) + " parameter was not found.");
  }

  // Get the type of odometry topic to use initially
  std::string source_odom_type;
  if (!GetParam(shared_from_this(), kInitialOdomName, source_odom_type)) {
    throw std::runtime_error(std::string(kInitialOdomName) + " parameter was not found.");
  }

  // Check if the set initial odom is valid
  auto found_odom = odom_topic_list.find(source_odom_type);
  if (found_odom == odom_topic_list.end()) {
    source_odom_type = odom_topic_list.begin()->first;
    RCLCPP_WARN_STREAM(this->get_logger(),
        kInitialOdomName
        << " parameter is invalid! Use [" << source_odom_type
        << "] source for now!");
  }

  // Obtain the parent-child frame names for the output odometry
  std::string odom_frame;
  GetOptionalParam(shared_from_this(), kOdomParamName, odom_frame, std::string(kDefaultOdomName));
  std::string odom_child_frame;
  GetOptionalParam(shared_from_this(), kOdomChildParamName, odom_child_frame, std::string(kDefaultOdomChildName));

  OdometryList odometry_list;
  for (const auto odom_topic : odom_topic_list) {
    odometry_list.insert(std::make_pair(odom_topic.first, Eigen::Affine3d::Identity()));
  }
  odometry_switcher_.reset(new OdometrySwitcher(source_odom_type, odometry_list));

  OdometryPublisher::Ptr publisher(
      new OdometryPublisher(shared_from_this(), odom_frame, odom_child_frame));

  /// Generate subscribers for each topic in the list
  for (const auto odom_topic : odom_topic_list) {
    OdometrySubscriber::Ptr subscriber(
        new OdometrySubscriber(shared_from_this(), odom_topic.first, odom_topic.second.get_value<std::string>(),
        odometry_switcher_, publisher));
    odom_subscribers_.push_back(subscriber);
  }

  switch_server_ =
      this->create_service<tmc_navigation_msgs::srv::OdometrySwitch>(kOdomSwitchServicetName,
      std::bind(&OdometrySwitcherNode::SwitchServiceCallback, this, _1, _2));
}


/// Odometry switch service callback
bool OdometrySwitcherNode::SwitchServiceCallback(
    tmc_navigation_msgs::srv::OdometrySwitch::Request::SharedPtr req,
    tmc_navigation_msgs::srv::OdometrySwitch::Response::SharedPtr res) {
  const std::string set_odom_type = req->odom_type.data;
  // Switch the type of odometry
  if (odometry_switcher_->SwitchSourceOdom(set_odom_type)) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Set to " << set_odom_type);
    res->is_success = true;
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Odom type is not in list.");
    res->is_success = false;
  }
}

/// Odometry subscriber class constructor
/// @param odom_type [I] Type of odometry
/// @param topic_name [I] Topic name
/// @param switcher [I] Switcher
/// @param publisher [I] Publisher
OdometrySubscriber::OdometrySubscriber(
    rclcpp::Node::SharedPtr node, const std::string& odom_type, const std::string& topic_name,
    const OdometrySwitcher::Ptr& switcher, const OdometryPublisher::Ptr& publisher)
    : odom_type_(odom_type),
      odometry_switcher_(switcher),
      odometry_publisher_(publisher) {
  subscriber_ = node->create_subscription<nav_msgs::msg::Odometry>(
      topic_name, 10,
      std::bind(&OdometrySubscriber::OdometryCallback, this, _1));
}

/// Odometry callback
/// @param msg [I] Odometry
void OdometrySubscriber::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  Eigen::Affine3d odom_pose;
  tf2::fromMsg(msg->pose.pose, odom_pose);
  Eigen::Affine3d output_odom_pose;
  if (odometry_switcher_->UpdateOdometry(odom_type_, odom_pose, output_odom_pose)) {
    nav_msgs::msg::Odometry odometry = *msg;
    odometry.pose.pose = tf2::toMsg(output_odom_pose);
    odometry_publisher_->PublishOdometry(odometry);
  }
}

/// Odometry publisher class constructor
/// @param odom_frame [I] Odometry frame name
/// @param odom_child_frame [I] Odometry child frame name
OdometryPublisher::OdometryPublisher(
    rclcpp::Node::SharedPtr node, const std::string& odom_frame, const std::string& odom_child_frame)
    : tf_broadcaster_(*node), odom_frame_(odom_frame), odom_child_frame_(odom_child_frame) {
  publisher_ = node->create_publisher<nav_msgs::msg::Odometry>(kPublishTopicName, 10);
}

/// Output odometry as TOPIC and TF
/// @param odometry [I] Odometry
void OdometryPublisher::PublishOdometry(const nav_msgs::msg::Odometry& odometry) {
  nav_msgs::msg::Odometry output_odom = odometry;
  output_odom.header.frame_id = odom_frame_;
  output_odom.child_frame_id = odom_child_frame_;
  publisher_->publish(output_odom);
  tf_broadcaster_.sendTransform(OdometryMsgToTransform(output_odom));
}

/// Odometry switching
/// @param odom_type [I] Type of odometry
/// @ret true: switch successful / false: switch failed
bool OdometrySwitcher::SwitchSourceOdom(const std::string& odom_type) {
  if (odom_list_.find(odom_type) == odom_list_.end()) {
    return false;
  }
  // Update the position to switch odometry
  const Eigen::Affine3d current_transform = GetCurrentOdometryTransform();
  base_transform_ = current_transform * odom_list_[odom_type].inverse();
  // Update source odometry
  source_odom_type_ = odom_type;
  return true;
}

/// Update odometry
/// @param odom_type [I] Type of odometry
/// @param odom_pose [I] Odometry position
/// @param output_odom_pose [O] Updated odometry position
/// @ret true: odometry has been updated / false: odometry has not been updated
bool OdometrySwitcher::UpdateOdometry(const std::string& odom_type,
                                      const Eigen::Affine3d& odom_pose,
                                      Eigen::Affine3d& output_odom_pose) {
  odom_list_[odom_type] = odom_pose;
  // Exit if not the source odometry
  if (odom_type != source_odom_type_) {
    return false;
  }
  // Convert and calculate the position based on the switch position reference
  output_odom_pose = GetCurrentOdometryTransform();
  return true;
}

/// Calculate the position of odometry
Eigen::Affine3d OdometrySwitcher::GetCurrentOdometryTransform() const {
  return base_transform_ * odom_list_.at(source_odom_type_);
}

}  // namespace tmc_odometry_switcher
