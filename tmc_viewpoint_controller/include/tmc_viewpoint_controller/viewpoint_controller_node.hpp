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
/// @file viewpoint_controller_node.hpp
/// @brief Node to control the viewpoint using the neck pan axis

#ifndef TMC_VIEWPOINT_CONTROLLER_VIEWPOINT_CONTROLLER_NODE_HPP_
#define TMC_VIEWPOINT_CONTROLLER_VIEWPOINT_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "tmc_viewpoint_controller/viewpoint_to_path.hpp"
#include "tmc_viewpoint_controller/viewpoint_to_tracking_target.hpp"

namespace tmc_viewpoint_controller {

class ViewpointControllerNode : public rclcpp::Node {
 public:
  explicit ViewpointControllerNode(const rclcpp::NodeOptions& options);
  virtual ~ViewpointControllerNode();
  /// Initialization
  void Init();
  /// Main processing
  void Run();

 private:
  /// Callback function
  void CallbackJointState(const sensor_msgs::msg::JointState::SharedPtr joint_states);
  /// Viewpoint control function On service
  void StartServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);
  /// Viewpoint control function On service
  void StopServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);
  /// Set control mode to Path (orient viewpoint towards own path)
  void SetViewpointModePathServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);
  /// Set control mode to Tracking (orient viewpoint towards target)
  void SetViewpointModeTrackingServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);

  /// Adjust command values considering neck axis limitations
  double NeckPanningFilter(const double command);
  /// Change viewpoint
  void ChangeViewpoint();

  /// Joint axis subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  /// Neck trajectory publisher
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_trajectory_pub_;
  /// tf buffer
  tf2_ros::Buffer tf_buffer_;
  /// tf listener
  tf2_ros::TransformListener tf_listener_;
  /// Function On service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service_;
  /// Function Off service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
  /// Viewpoint mode (path) switch service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_viewpoint_mode_path_service_;
  /// Viewpoint mode (tracking) switch service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_viewpoint_mode_tracking_service_;
  /// Current pan axis information
  double current_neck_pan_angle_;
  /// On/Off flag for function
  bool enable_view_ctrl_;
  /// Maximum rotation amount per cycle
  double max_rotation_once_;
  /// Right turn mechanical limit
  double head_pan_min_;
  /// Left turn mechanical limit
  double head_pan_max_;
  /// Neck pan axis name
  std::string neck_pan_name_;
  /// Neck tilt axis name
  std::string neck_tilt_name_;
  /// Fixed neck tilt angle
  double fixed_neck_tilt_;
  /// Drive cycle
  double rate_;
  /// Path direction viewpoint calculation
  ViewpointToPath::Ptr viewpoint_to_path_;
  /// Target direction viewpoint calculation
  ViewpointToTrackingTarget::Ptr viewpoint_to_tracking_target_;
  /// Map frame name
  std::string map_frame_;
  /// Cart frame name
  std::string base_frame_;
  /// Viewpoint planning mode
  uint32_t viewpoint_control_mode_;
};
}  // end namespace tmc_viewpoint_controller

#endif  // TMC_VIEWPOINT_CONTROLLER_VIEWPOINT_CONTROLLER_NODE_HPP_
