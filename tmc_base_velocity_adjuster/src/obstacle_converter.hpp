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
/// @File obstacle_converter.hpp
/// @brief Convert obstacle topics into a data format processed by the department
#ifndef TMC_BASE_VELOCITY_ADJUSTER_OBSTACLE_CONVERTER_HPP_
#define TMC_BASE_VELOCITY_ADJUSTER_OBSTACLE_CONVERTER_HPP_
#include <map>
#include <memory>
#include <string>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "param.hpp"

namespace { // NOLINT
// Robot center frame name
constexpr const char* const kBaseFrameName = "base_link";
// Downsample interval parameter name
constexpr const char* const kFilterLeafSizeName = "filter_leaf_size";
// Default value of downsample interval [m]
constexpr double kFilterLeafSizeDefault = 0.025;
// Parameter name for the radius of the point cloud range to filter and save
constexpr const char* const kFilterAreaName = "filter_area_radius";
// Default value of the radius of the point cloud range to filter and save [m]
constexpr double kFilterAreaDefault = 1.5;
}  // anonymous namespace

namespace tmc_base_velocity_adjuster {
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

/// @brief Obstacle conversion data transformation class Converts from Ros type to data type used for control
/// @tparam RosMsg Input Ros message type
/// @tparam Data Obstacle data type used for optimization internally
template<class RosMsg, class Data>
class ObstacleConverter {};

/// @brief Conversion class from PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
template<>
class ObstacleConverter<sensor_msgs::msg::PointCloud2, pcl::PointCloud<pcl::PointXYZ>> {
 public:
  /// @brief Constructor
  /// @param [in] node Node
  explicit ObstacleConverter(const rclcpp::Node::SharedPtr node)
      : tf_buffer_(std::make_shared<rclcpp::Clock>(rclcpp::Clock(RCL_ROS_TIME))),
        tf_listener_(new tf2_ros::TransformListener(tf_buffer_)) {
    // Parameter acquisition
    GetOptionalParam(node, "base_frame", base_frame_, std::string(kBaseFrameName));

    std::map<std::string, rclcpp::Parameter> converter_params;
    GetGroupParam(node, "obstacle_converter", converter_params);

    double filter_leaf_size = 0.0;
    GetOptionalParam(converter_params, kFilterLeafSizeName, filter_leaf_size, kFilterLeafSizeDefault);
    if (filter_leaf_size <= 0.0) {
      RCLCPP_WARN(node->get_logger(), "Parameter [%s] is not set or invalid. Use default value",
          kFilterLeafSizeName);
      filter_leaf_size = kFilterLeafSizeDefault;
    }
    voxel_filter_.setLeafSize(filter_leaf_size, filter_leaf_size, filter_leaf_size);

    GetOptionalParam(converter_params, kFilterAreaName, filter_area_radius_, kFilterAreaDefault);
    if (filter_area_radius_ <= 0.0) {
      RCLCPP_WARN(node->get_logger(), "Parameter [%s] is not set or invalid. Use default value",
          kFilterAreaName);
      filter_area_radius_ = kFilterAreaDefault;
    }
  }

  /// @brief Obstacle conversion data transformation Convert to robot-centered coordinate system, reduce points, and save
  /// @param [in] input Ros type point cloud
  /// @param [out] output Pcl type point cloud
  void Convert(const sensor_msgs::msg::PointCloud2::ConstPtr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {
    output->points.clear();
    // Convert to robot center
    // Directly convert PointCloud2 for speed optimization
    auto transformed_input = std::make_shared<sensor_msgs::msg::PointCloud2>();
    try {
      auto base_to_sensor = tf_buffer_.lookupTransform(base_frame_, input->header.frame_id,
                                                       input->header.stamp,
                                                       rclcpp::Duration::from_seconds(1.0));
      pcl_ros::transformPointCloud(tf2::transformToEigen(base_to_sensor.transform).matrix().cast<float>(),
                                   *input,
                                   *transformed_input);
      transformed_input->header.frame_id = base_frame_;
    } catch (tf2::TransformException& ex) {
      auto clock = rclcpp::Clock(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("base_velocity_adjuster"), clock, 5000,
          "Cannot transform point cloud : %s", ex.what());
    }

    auto input_cloud = PointCloud::Ptr(new PointCloud());
    pcl::fromROSMsg(*transformed_input, *input_cloud);

    // Downsample using voxel filter
    PointCloud::Ptr down_sampled_cloud(new PointCloud());
    voxel_filter_.setInputCloud(input_cloud);
    voxel_filter_.filter(*down_sampled_cloud);

    // Extract point cloud within a certain distance
    for (auto it = std::begin(down_sampled_cloud->points); it != std::end(down_sampled_cloud->points); ++it) {
      Eigen::Vector3d point_pos(it->x, it->y, 0.0);
      if (point_pos.norm() < filter_area_radius_) {
        output->points.push_back(*it);
      }
    }
  }

 private:
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  // Voxel filter
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  // Radius of the point cloud range to filter and save [m]
  double filter_area_radius_;
  // Frame name of the base coordinate
  std::string base_frame_;
};

}  // namespace tmc_base_velocity_adjuster

#endif  // TMC_BASE_VELOCITY_ADJUSTER_OBSTACLE_CONVERTER_HPP_
