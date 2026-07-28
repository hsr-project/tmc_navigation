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
/// @file point_cloud_accumulator_node.hpp
/// @brief Past Point Cloud Accumulation Node Class
#ifndef TMC_POINT_CLOUD_ACCUMULATOR_POINT_CLOUD_ACCUMULATOR_NODE_HPP_
#define TMC_POINT_CLOUD_ACCUMULATOR_POINT_CLOUD_ACCUMULATOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_listener.h>

#include "common.hpp"
#include "point_cloud_accumulator.hpp"

namespace tmc_point_cloud_accumulator {

class PointCloudAccumulatorNode : public rclcpp::Node {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  explicit PointCloudAccumulatorNode(const rclcpp::NodeOptions& options);
  // Destructor
  virtual ~PointCloudAccumulatorNode() {}
  // Initialization
  void Init();

 private:
  // Point Cloud Callback Function
  void PointCloudCallback(const PointCloud2Ptr msg);

  // Service Callback to Start Point Cloud Accumulation
  void StartAccumulateServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);
  // Service Callback to Stop Point Cloud Accumulation
  void StopAccumulateServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);
  // Callback Function for Main Processing Execution Timer
  void NodeActionTimerCallback();

  // Merge Current Point Cloud with Accumulated Point Cloud
  PointCloudPtr MergePointCloudInScanArea(const PointCloudPtr& input_cloud);

  // Publish Point Cloud
  void PublishPointCloud(const PointCloudPtr& cloud);

  // Save Point Cloud
  void SavePointCloud(const PointCloudPtr& cloud);

  // Transform Point Cloud Coordinates to Specified Frame Reference
  PointCloudPtr TransformPointCloud(const PointCloudPtr& input_cloud, const std::string& base_frame);

  // Retrieve and Store Parameters
  void LoadFromROSParam();

  // Input Point Cloud Subscriber
  rclcpp::Subscription<PointCloud2>::SharedPtr point_cloud_subscriber_;
  // Output Point Cloud Publisher
  rclcpp::Publisher<PointCloud2>::SharedPtr obstacle_point_cloud_publisher_;
  // Service to Start Point Cloud Accumulation
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_accumulate_service_;
  // Service to Stop Point Cloud Accumulation
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_accumulate_service_;
  // Timer for Main Processing Execution
  rclcpp::TimerBase::SharedPtr node_action_timer_;

  // Point Cloud Accumulation Class
  std::shared_ptr<PointCloudAccumulator> accumulator_;

  // Coordinate Transformation Class
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Reference Frame Name
  std::string map_frame_;
  // Odometry Frame Name
  std::string odom_frame_;
  // Robot Base Frame Name
  std::string base_frame_;

  // Subscribed Point Cloud
  PointCloud2Ptr subscribed_cloud_;
  // Current Point Cloud
  PointCloudPtr current_cloud_;
  // Current Robot Odometry
  Eigen::Affine3d current_odometry_;


  // Radius Distance [m] from Bot Center to Exclude from Point Cloud
  double trimming_data_radius_;
  // Lower Height Limit [m] for Accumulated Point Cloud
  double bottom_of_valid_space_;
  // Upper Height Limit [m] for Accumulated Point Cloud
  double top_of_valid_space_;
  // Point Interval [m] for Downsampling
  double voxel_leaf_size_;
  // Minimum Radius Threshold [m] from Robot for Saving Point Cloud
  double min_saved_area_radius_;
  // Maximum Radius Threshold [m] from Robot for Saving Point Cloud
  double max_saved_area_radius_;
  // Field of View [deg] to Cut Past Point Cloud Data Based on Robot Front Reference
  double cut_point_cloud_angle_;
  // Point Cloud Noise Filter Search Range [m]
  double noise_filter_radius_search_;
  // Number of Points within Point Cloud Noise Filter Search Range
  int32_t noise_filter_neighbors_;

  // On/Off Flag for This Functionality
  bool enable_function_;
};
}  // namespace tmc_point_cloud_accumulator

#endif  // TMC_POINT_CLOUD_ACCUMULATOR_POINT_CLOUD_ACCUMULATOR_NODE_HPP_
