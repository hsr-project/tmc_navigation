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
/// @file point_cloud_merger_node.cpp
/// @brief Node for merging two point clouds
#include <chrono>
#include <string>
#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tmc_point_cloud_accumulator/param.hpp"
#include "tmc_point_cloud_accumulator/point_cloud_merger_node.hpp"

namespace {
/// Buffer size of the topic
const int32_t kTopicBufferSize = 1;
/// Threshold for tf processing wait time [s]
const double kTfWaitDuration = 1.0;
/// Processing cycle [Hz]
const double kLoopHz = 10;  // Do NOT set the kLoopHz to zero or less
/// Default valid measurement range of point cloud 1 [m]
const double kDefaultCloud1Range = 30.0;
/// Default valid measurement range of point cloud 2 [m]
const double kDefaultCloud2Range = 30.0;
/// Console log message display interval [ms]
const int32_t kWarnLogIndicatePeriod = 5000;
/// TF log message display interval [ms]
const int32_t kTFLogIndicatePeriod = 5000;
/// Timeout for point cloud update determination [s]
const double kUpdatePointCloudTimeout = 10.0;
/// Parameter name for valid measurement range (point cloud 1)
const char* const kCloud1RangeParamName = "cloud_1_range";
/// Parameter name for valid measurement range (point cloud 2)
const char* const kCloud2RangeParamName = "cloud_2_range";

void CopyMetaData(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in, pcl::PointCloud<pcl::PointXYZ>::Ptr& out) {
  out->header = in->header;
  out->is_dense = in->is_dense;
  out->height = 1;
}
}  // namespace


namespace tmc_point_cloud_accumulator {
using std::chrono::milliseconds;
using std::placeholders::_1;
// Constructor
PointCloudMergerNode::PointCloudMergerNode(const rclcpp::NodeOptions& options)
    : Node("point_cloud_merger", options),
      use_cloud_1_(true),
      use_cloud_2_(true),
      cloud_1_range_(kDefaultCloud1Range),
      cloud_2_range_(kDefaultCloud2Range) {}

// Initialization
void PointCloudMergerNode::Init() {
  tf_buffer_ =
      std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Point cloud initialization
  cloud_1_ = PointCloud2Ptr(new PointCloud2);
  cloud_2_ = PointCloud2Ptr(new PointCloud2);
  // Parameter retrieval
  GetOptionalParam(shared_from_this(), kCloud1RangeParamName, cloud_1_range_, kDefaultCloud1Range);
  GetOptionalParam(shared_from_this(), kCloud2RangeParamName, cloud_2_range_, kDefaultCloud2Range);

  // Set subscriber for point cloud 1
  cloud_1_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_point_cloud_1", kTopicBufferSize, std::bind(&PointCloudMergerNode::PointCloud1Callback, this, _1));

  // Set subscriber for point cloud 2
  cloud_2_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_point_cloud_2", kTopicBufferSize, std::bind(&PointCloudMergerNode::PointCloud2Callback, this, _1));

  // Set publisher for output point cloud
  merged_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "merged_point_cloud", kTopicBufferSize);

  // Set main processing execution timer
  node_action_timer_ = this->create_wall_timer(milliseconds(static_cast<int32_t>(1000 / kLoopHz)),
      std::bind(&PointCloudMergerNode::NodeActionTimerCallback, this));
}

// Callback function for point cloud 1
void PointCloudMergerNode::PointCloud1Callback(const PointCloud2SharedPtr msg) {
  if (!msg->data.empty()) {
    cloud_1_->data.clear();
    *cloud_1_ = *msg;
  }
}

// Callback function for point cloud 2
void PointCloudMergerNode::PointCloud2Callback(const PointCloud2SharedPtr msg) {
  if (!msg->data.empty()) {
    cloud_2_->data.clear();
    *cloud_2_ = *msg;
  }
}

// Timer callback function for executing main processing
void PointCloudMergerNode::NodeActionTimerCallback() {
  PointCloud2Ptr trimmed_cloud_1(new PointCloud2);
  PointCloud2Ptr trimmed_cloud_2(new PointCloud2);
  PointCloud2Ptr transformed_cloud_2(new PointCloud2);
  PointCloud2Ptr cloud_out(new PointCloud2);

  // Merge two point clouds
  if ((use_cloud_1_ && !cloud_1_->data.empty()) && (use_cloud_2_ && !cloud_2_->data.empty())) {
    // Trim point cloud
    trimmed_cloud_1 = TrimPointCloud(cloud_1_, cloud_1_range_);
    trimmed_cloud_2 = TrimPointCloud(cloud_2_, cloud_2_range_);
    // Transform cloud_2_ to the frame of cloud_1_
    transformed_cloud_2 = TransformPointCloud(trimmed_cloud_2, trimmed_cloud_1->header.frame_id);
    // Merge point clouds (trimmed_cloud_1 += transformed_cloud_2)
    if (!pcl::concatenatePointCloud(*trimmed_cloud_1, *transformed_cloud_2, *cloud_out)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to merger point cloud");
    }
    // Publish merged point cloud
    merged_cloud_publisher_->publish(*cloud_out);

    // Clear buffer
    cloud_1_->data.clear();
    cloud_2_->data.clear();
    trimmed_cloud_1->data.clear();
    trimmed_cloud_2->data.clear();
    transformed_cloud_2->data.clear();
    cloud_out->data.clear();
    // If one or both point clouds are not subscribed
    // Output a warning log
  } else if ((use_cloud_1_ && use_cloud_2_) && (cloud_1_->data.empty() || cloud_2_->data.empty())) {
    // Compare the time of the last received point cloud with the current time
    // If a certain time has passed, output a log
    rclcpp::Duration elapsed_time_cloud1 = rclcpp::Clock(RCL_ROS_TIME).now() - rclcpp::Time(cloud_1_->header.stamp);
    rclcpp::Duration elapsed_time_cloud2 = rclcpp::Clock(RCL_ROS_TIME).now() - rclcpp::Time(cloud_2_->header.stamp);
    if (elapsed_time_cloud1 > rclcpp::Duration::from_seconds(kUpdatePointCloudTimeout) ||
        elapsed_time_cloud2 > rclcpp::Duration::from_seconds(kUpdatePointCloudTimeout)) {
      auto clock = rclcpp::Clock(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(this->get_logger(), clock, kWarnLogIndicatePeriod,
          "However two point clouds are required, either or both point cloud was not subscribed. "
          "Point_cloud_merger could not publish merged point cloud.");
    }
    // Use only cloud_1
  } else if ((use_cloud_1_ && !cloud_1_->data.empty()) && !use_cloud_2_) {
    trimmed_cloud_1 = TrimPointCloud(cloud_1_, cloud_1_range_);
    merged_cloud_publisher_->publish(*trimmed_cloud_1);
    cloud_1_->data.clear();
    trimmed_cloud_1->data.clear();
    // Use only cloud_2
  } else if ((use_cloud_2_ && !cloud_2_->data.empty()) && !use_cloud_1_) {
    trimmed_cloud_2 = TrimPointCloud(cloud_2_, cloud_2_range_);
    merged_cloud_publisher_->publish(*trimmed_cloud_2);
    cloud_2_->data.clear();
    trimmed_cloud_2->data.clear();
  }
}

// Transform point cloud coordinates to the specified frame of reference
PointCloud2Ptr PointCloudMergerNode::TransformPointCloud(PointCloud2Ptr& input_cloud,
                                                         const std::string& target_frame) {
  PointCloud2Ptr transformed_cloud(new PointCloud2);
  auto clock = rclcpp::Clock(RCL_ROS_TIME);
  try {
    const bool transform_is_found = tf_buffer_->canTransform(target_frame,
        input_cloud->header.frame_id,
        rclcpp::Time(input_cloud->header.stamp),
        rclcpp::Duration::from_seconds(kTfWaitDuration));
    if (transform_is_found) {
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
          target_frame, input_cloud->header.frame_id, rclcpp::Time(input_cloud->header.stamp),
          rclcpp::Duration::from_seconds(kTfWaitDuration));
      tf2::doTransform(*input_cloud, *transformed_cloud, transform_stamped);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), clock, kTFLogIndicatePeriod,
          "wait for transform timeout...");
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, kTFLogIndicatePeriod,
        "transform error : %s", ex.what());
  }
  return transformed_cloud;
}

// Format point cloud
PointCloud2Ptr PointCloudMergerNode::TrimPointCloud(const PointCloud2Ptr& input_cloud, double valid_range) {
  PointCloud2Ptr output_cloud(new PointCloud2());
  PointCloudPtr input_pcl_cloud(new PointCloud());
  PointCloudPtr output_pcl_cloud(new PointCloud());

  pcl::fromROSMsg(*input_cloud, *input_pcl_cloud);
  uint32_t cnt = 0;
  Eigen::Vector3d point_3d = Eigen::Vector3d::Zero();
  CopyMetaData(input_pcl_cloud, output_pcl_cloud);

  // Use only points within valid_range distance
  BOOST_FOREACH (const pcl::PointXYZ& p, input_pcl_cloud->points) {
    double distance = p.x * p.x + p.y * p.y + p.z * p.z;
    if (distance < valid_range * valid_range) {
      output_pcl_cloud->points.push_back(p);
      ++cnt;
    }
  }
  output_pcl_cloud->width = cnt;
  pcl::toROSMsg(*output_pcl_cloud, *output_cloud);

  return output_cloud;
}
}  // namespace tmc_point_cloud_accumulator
