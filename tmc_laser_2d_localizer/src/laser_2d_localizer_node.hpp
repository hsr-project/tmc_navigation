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
/// @file     laser_2d_localizer_node.hpp
/// @brief    Definition of laser_2d_localizer
/// @version  0.x.x
/// @author   Yoshiaki Asahara
/// @date     2011.xx.xx
/// @since    2011.xx.xx
/// @note     Applied for Partner-Robot Coding Rule(Ver:x.xx)

#ifndef TMC_LASER_2D_LOCALIZER_LASER_2D_LOCALIZER_NODE_HPP_
#define TMC_LASER_2D_LOCALIZER_LASER_2D_LOCALIZER_NODE_HPP_

#include <stdint.h>
#include <memory>
#include <string>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tmc_navigation_msgs/srv/bool_response.hpp>
#include <tmc_navigation_msgs/srv/set_localization_score_limit.hpp>
#include <tmc_pose_2d_lib/distance_map.hpp>
#include <tmc_pose_2d_lib/pose_2d.hpp>
#include <tmc_pose_2d_lib/ros_if.hpp>
#include "laser_2d_mcl.hpp"

namespace tmc_laser_2d_localizer {
using tmc_pose_2d_lib::Pose2d;
using tmc_pose_2d_lib::GetPose2dFromRosMsg;

/// Default frame name of the global coordinate system
const char* const kGlobalFrameId = "map";

/**
    \brief  Laser2dLocalizer class
    \par
    Creates self-localization estimation data using 2D LRF data.
    Uses the self-localization library laser_2d_mcl_lib.c.
*/

class Laser2dLocalizerNode : public rclcpp::Node {
 public:
  explicit Laser2dLocalizerNode(const rclcpp::NodeOptions& options);
  ~Laser2dLocalizerNode();
  // Initialization
  void Init();

 private:
  void SetParams();
  void InitializePublishers();
  void InitializeServices();
  void PublishInitialPose();
  void InitializeSubscribers();
  void StaticGridMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr distance_map_msg);
  void PointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msg);
  void CorrectPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr correct_pose_msg);
  void StartLocalizedPose(
      std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);
  void StopLocalizedPose(
      std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);
  void CheckLocalizerRunning(
      tmc_navigation_msgs::srv::BoolResponse::Request::SharedPtr req,
      tmc_navigation_msgs::srv::BoolResponse::Response::SharedPtr res);
  void SetLocalizationScore(
      tmc_navigation_msgs::srv::SetLocalizationScoreLimit::Request::SharedPtr req,
      tmc_navigation_msgs::srv::SetLocalizationScoreLimit::Response::SharedPtr res);
  void UpdateOdometry(const Pose2d& odom);
  void TransformPointCloud(const std::string& frame_id,
                           const sensor_msgs::msg::PointCloud2& input_cloud,
                           sensor_msgs::msg::PointCloud2& output_cloud);


  /// Switch is triggered by odometry movement threshold judgment
  bool run_mcl_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_distance_map_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr correct_pose_subscriber_;
  /// Publisher of self-localization estimation results
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr laser_2d_pose_publisher_;
  /// Publisher of current particle status results
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr particle_positions_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr score_publisher_;

  bool is_publisher_initialized_;
  /// MCL algorithm core
  Laser2dMcl laser_2d_mcl_;
  /// frame_id of the map
  std::string frame_id_;
  /// Whether to PUBLISH laser_2d_pose or not
  bool publish_laser_2d_pose_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_localized_pose_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_localized_pose_;
  rclcpp::Service<tmc_navigation_msgs::srv::BoolResponse>::SharedPtr check_localizer_running_;
  rclcpp::Service<tmc_navigation_msgs::srv::SetLocalizationScoreLimit>::SharedPtr set_localization_score_limit_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string odometry_tf_name_;
  std::string base_tf_name_;
  std::string robot_tf_name_;


  // Distance map
  std::shared_ptr<DistanceMap> distance_map_;
  /// Accumulated movement amount
  double sum_distance_;
  /// Accumulated rotation amount
  double sum_angle_;
  /// Previous odometry
  Pose2d previous_odom_;
  /// Whether odometry has been received or not
  bool is_first_odometry_received_;
  /// Odometry movement limit distance [m]
  double max_odom_distance_threshold_;
  /// Odometry rotation limit amount [rad]
  double max_odom_angle_threshold_;
};
}  // namespace tmc_laser_2d_localizer

#endif  // TMC_LASER_2D_LOCALIZER_LASER_2D_LOCALIZER_NODE_HPP_
