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
/// @file point_cloud_merger_node.hpp
/// @brief Node for merging two point clouds
#ifndef TMC_POINT_CLOUD_ACCUMULATOR_POINT_CLOUD_MERGER_NODE_HPP_
#define TMC_POINT_CLOUD_ACCUMULATOR_POINT_CLOUD_MERGER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "common.hpp"

namespace tmc_point_cloud_accumulator {

class PointCloudMergerNode : public rclcpp::Node {
 public:
  // Constructor
  explicit PointCloudMergerNode(const rclcpp::NodeOptions& options);
  // Destructor
  virtual ~PointCloudMergerNode() {}
  // Initialization
  void Init();

 private:
  // Callback function for point cloud 1
  void PointCloud1Callback(const PointCloud2SharedPtr msg);

  // Callback function for point cloud 2
  void PointCloud2Callback(const PointCloud2SharedPtr msg);

  // Callback function for the main processing timer
  void NodeActionTimerCallback();

  // Point cloud coordinate transformation
  PointCloud2Ptr TransformPointCloud(PointCloud2Ptr& input_cloud, const std::string& target_frame);

  // Point cloud formatting
  PointCloud2Ptr TrimPointCloud(const PointCloud2Ptr& input_cloud, double valid_range);

  // Subscriber for input point cloud 1
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_1_subscriber_;
  // Subscriber for input point cloud 2
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_2_subscriber_;
  // Publisher for output point cloud
  rclcpp::Publisher<PointCloud2>::SharedPtr merged_cloud_publisher_;
  // Timer for executing main processing
  rclcpp::TimerBase::SharedPtr node_action_timer_;

  // Coordinate transformation class
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Subscribed point cloud 1
  PointCloud2Ptr cloud_1_;
  // Subscribed point cloud 2
  PointCloud2Ptr cloud_2_;

  // TODO(syuuhei_shiro): use_cloud_1,2は現状true固定。パラメータ公開し、変更をコールバックで受け取れるようにする
  // Whether to use point cloud 1
  bool use_cloud_1_;
  // Whether to use point cloud 2
  bool use_cloud_2_;
  // Valid measurement range [m] (point cloud 1)
  double cloud_1_range_;
  // Valid measurement range [m] (point cloud 2)
  double cloud_2_range_;
};
}  // namespace tmc_point_cloud_accumulator

#endif  // TMC_POINT_CLOUD_ACCUMULATOR_POINT_CLOUD_MERGER_NODE_HPP_
