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
/// @file     pose_integrator_node.hpp
/// @brief    Integrate multiple self-positioning results (definition part)
/// @version  0.2.0
/// @author   Takao Yasuda
/// @author   Applied for Partner-Robot Coding Rule(Ver:x.xx)
/// @date     2012.05.08

#ifndef TMC_POSE_INTEGRATOR_POSE_INTEGRATOR_NODE_HPP_
#define TMC_POSE_INTEGRATOR_POSE_INTEGRATOR_NODE_HPP_

#include <memory>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "pose_integrator.hpp"

/// Namespace (tmc_pose_integrator)
namespace tmc_pose_integrator {

/// Class (PoseIntegratorNode)
class PoseIntegratorNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit PoseIntegratorNode(const rclcpp::NodeOptions& options);
  /// Destructor
  ~PoseIntegratorNode();
  /// Initialization
  void Init();
  /// Update global self-position
  void UpdateGlobalPose();

 private:
  /// Callback (laser self-positioning)
  void CallbackLaser2dPose_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr laser_2d_msg);
  /// Callback (odometry)
  void CallbackOdometry_(const nav_msgs::msg::Odometry::SharedPtr odometry_msg);
  /// Publish self-position integration results to topic & update tf frame
  void SendGlobalPose_(const Pose2d& pose);
  /// callback for tf broadcast
  void CallbackTfBroadcast_();
  /// Get odom from tf
  void GetOdometryFromTf(void);
  /// Flag to check if laser_2d_pose subscription is the first time
  bool is_first_laser_2d_pose_;
  /// Flag to use URG for map generation
  bool is_inverted_urg_;
  /// node transfers tf(map->odom) only when odometry is updated.
  bool is_odom_updated_;
  /// Specify the initial installation position of the robot's upper arm relative to the robot's orientation (deg)
  double robot_base_tf_yaw_;
  /// Timestamp for checking laser_2d_pose data update
  double pre_laser_2d_pose_time_;
  /// Frame name of the global coordinate system
  std::string global_frame_id_;
  /// Frame name of the robot cart
  std::string base_frame_id_;
  /// Frame name of the odometry
  std::string odom_frame_id_;
  /// Self-position integration class
  PoseIntegrator::Ptr pose_integrator_;
  /// Broadcaster for self-position frame update
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  /// timer for tf broadcast
  rclcpp::TimerBase::SharedPtr timer_;
  /// Periodic controller
  std::shared_ptr<rclcpp::Rate> rate_;
  /// Subscriber (laser self-positioning)
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscribe_laser2d_pose_;
  /// Publisher (self-position estimation)
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_publisher_;
  /// Difference between map origin and global origin
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  /// Timestamp of the latest published global_pose tf
  double latest_global_pose_tf_stamp_;
  /// Latest published global_pose
  Pose2d global_pose_;
  /// odom frame posture based on map
  tf2::Stamped<tf2::Transform> odom_to_map_;
};  // class PoseIntegratorNode

}  // namespace tmc_pose_integrator

#endif  // TMC_POSE_INTEGRATOR_POSE_INTEGRATOR_NODE_HPP_
