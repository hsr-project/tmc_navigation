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
/// @file point_cloud_accumulator_node.cpp
/// @brief Past Point Cloud Accumulation Node Class
#include <chrono>
#include <string>
#include <vector>

#include "tmc_point_cloud_accumulator/param.hpp"
#include "tmc_point_cloud_accumulator/point_cloud_accumulator_node.hpp"

namespace {
/// Topic Buffer Size
const int32_t kTopicBufferSize = 1;

/// Threshold for tf processing wait time [s]
const double kTfWaitDuration = 1.0;
/// Processing Cycle [ms]
const int32_t kLoopCycle = 100;

/// Reference Frame Name
const char* const kMapFrame = "map";
/// Odometry Frame Name
const char* const kOdomFrame = "odom";
/// Robot's Base Frame Name
const char* const kBaseFrame = "base_link";

/// Lower Limit of Point Cloud Height to Handle [m]
const double kBottomOfValidSpace = 0.15;
/// Upper Limit of Point Cloud Height to Handle [m]
const double kTopOfValidSpace = 1.5;
/// Point Interval for Point Cloud Downsampling [m]
const double kVoxelLeafSize = 0.025;
/// Minimum Radius Threshold of Distance from Robot When Saving Point Cloud [m]
const double kMinSavedAreaRadius = 0.2;
/// Maximum Radius Threshold of Distance from Robot When Saving Point Cloud [m]
const double kMaxSavedAreaRadius = 3.0;
/// Field of View Angle [deg] to Cut Past Point Cloud Data Based on Robot's Front Reference
const double kCutPointCloudAngle = 5.0;
/// Duration to Retain Past Point Cloud [s]
const double kPointCloudKeepPeriod = 6.0;
/// Radius of Distance from Robot Center to Exclude from Acquired Point Cloud [m]
const double kTrimmingDataRadius = 0.3;
/// Log Message Issuance Cycle [ms]
const int32_t kConsoleMessageIndicatePeriod = 5000;

/// Point Cloud Noise Filter Parameter Search Range [m]
/// Details at http://pointclouds.org/documentation/tutorials/remove_outliers.php
const double kNoiseFilterRadiusSearch = 0.1;
/// Point Cloud Noise Filter Parameter Number of Points Within Search Range
const int32_t kNoiseFilterNeighbors = 1;
}  // namespace


namespace tmc_point_cloud_accumulator {
using std::chrono::milliseconds;
using std::placeholders::_1;
using std::placeholders::_2;
// Perform Downsampling to Eliminate Duplicates
PointCloudPtr PointCloudDownSampling(const PointCloudPtr& input_cloud, const double voxel_leaf_size,
                                     const double noise_filter_radius, const int32_t noise_filter_neighbors) {
  PointCloudPtr output_cloud(new PointCloud());
  if (!input_cloud->points.empty()) {
    // Downsampling Filter Object
    pcl::VoxelGrid<pcl::PointXYZ> down_sample_filter;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_remover;

    PointCloudPtr down_sampled_cloud(new PointCloud());
    down_sample_filter.setInputCloud(input_cloud);
    down_sample_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    down_sample_filter.filter(*down_sampled_cloud);

    outlier_remover.setInputCloud(down_sampled_cloud);
    outlier_remover.setRadiusSearch(noise_filter_radius);
    outlier_remover.setMinNeighborsInRadius(noise_filter_neighbors);
    outlier_remover.filter(*output_cloud);
  }
  return output_cloud;
}
/// PointCloudAccumulatorNode Class
// Constructor
PointCloudAccumulatorNode::PointCloudAccumulatorNode(const rclcpp::NodeOptions& options)
    : Node("point_cloud_accumulator", options), enable_function_(true) {}

// Initialization
void PointCloudAccumulatorNode::Init() {
  tf_buffer_ =
      std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialization of Point Cloud Accumulation Class
  accumulator_ = std::shared_ptr<PointCloudAccumulator>(new PointCloudAccumulator(this->get_clock()));

  // Point Cloud Initialization
  subscribed_cloud_ = PointCloud2Ptr(new PointCloud2());
  current_cloud_.reset(new PointCloud());

  // Retrieve and Store Parameters
  LoadFromROSParam();

  // Set Subscriber for Point Cloud
  point_cloud_subscriber_ =
      this->create_subscription<PointCloud2>("input_point_cloud", kTopicBufferSize,
      std::bind(&PointCloudAccumulatorNode::PointCloudCallback, this, _1));

  // Set Publisher for Output Point Cloud
  obstacle_point_cloud_publisher_ =
      this->create_publisher<PointCloud2>("accumulated_point_cloud", kTopicBufferSize);

  // Set Service to Start Point Cloud Accumulation
  start_accumulate_service_ = this->create_service<std_srvs::srv::Empty>("start_accumulate_point_cloud",
      std::bind(&PointCloudAccumulatorNode::StartAccumulateServiceCallback, this, _1, _2));

  // Set Service to Stop Point Cloud Accumulation
  stop_accumulate_service_ = this->create_service<std_srvs::srv::Empty>("stop_accumulate_point_cloud",
      std::bind(&PointCloudAccumulatorNode::StopAccumulateServiceCallback, this, _1, _2));

  // Set Timer for Main Processing Execution
  node_action_timer_ = this->create_wall_timer(milliseconds(kLoopCycle),
      std::bind(&PointCloudAccumulatorNode::NodeActionTimerCallback, this));
}


// Retrieve and Store Parameters
void PointCloudAccumulatorNode::LoadFromROSParam() {
  // Reference Frame Name
  GetOptionalParam(shared_from_this(), "map_frame_name", map_frame_, std::string(kMapFrame));
  // Odometry Frame Name
  GetOptionalParam(shared_from_this(), "odom_frame_name", odom_frame_, std::string(kOdomFrame));
  // Robot's Base Frame Name
  GetOptionalParam(shared_from_this(), "base_frame_name", base_frame_, std::string(kBaseFrame));

  // Upper and Lower Limits of Point Cloud Height to Handle
  GetOptionalParam(shared_from_this(), "bottom_of_valid_space", bottom_of_valid_space_, kBottomOfValidSpace);
  GetOptionalParam(shared_from_this(), "top_of_valid_space", top_of_valid_space_, kTopOfValidSpace);
  // Specify Default Range if Parameters are Invalid
  if ((bottom_of_valid_space_ <= 0.0) || (top_of_valid_space_ <= bottom_of_valid_space_)) {
    RCLCPP_WARN(this->get_logger(),
        "Parameter 'bottom_of_valid_space' and 'top_of_valid_space' are invalid. Use default value [%lf] and [%lf].",
        kBottomOfValidSpace, kTopOfValidSpace);
    bottom_of_valid_space_ = kBottomOfValidSpace;
    top_of_valid_space_ = kTopOfValidSpace;
  }

  // Point Interval for Point Cloud Downsampling
  GetOptionalParam(shared_from_this(), "voxel_leaf_size", voxel_leaf_size_, kVoxelLeafSize);
  // Specify Default Value if Parameters are Invalid
  if (voxel_leaf_size_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Parameter 'voxel_leaf_size' is invalid. Use default value [%lf].",
        kVoxelLeafSize);
    voxel_leaf_size_ = kVoxelLeafSize;
  }

  // Distance Range from Robot When Saving Point Cloud
  GetOptionalParam(shared_from_this(), "min_saved_area_radius", min_saved_area_radius_, kMinSavedAreaRadius);
  GetOptionalParam(shared_from_this(), "max_saved_area_radius", max_saved_area_radius_, kMaxSavedAreaRadius);
  // Specify Default Range if Parameters are Invalid
  if ((min_saved_area_radius_ <= 0.0) ||
      (max_saved_area_radius_ <= min_saved_area_radius_)) {
    RCLCPP_WARN(this->get_logger(),
        "Parameter 'min_saved_area_radius' and 'max_saved_area_radius' are invalid. Use default value [%lf] "
        "and [%lf].",
        kMinSavedAreaRadius, kMaxSavedAreaRadius);
    min_saved_area_radius_ = kMinSavedAreaRadius;
    max_saved_area_radius_ = kMaxSavedAreaRadius;
  }

  // Field of View Angle to Cut Past Point Cloud Data Based on Robot's Front Reference
  GetOptionalParam(shared_from_this(), "cut_point_cloud_angle", cut_point_cloud_angle_, kCutPointCloudAngle);
  if (cut_point_cloud_angle_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Parameter 'cut_point_cloud_angle' is invalid. Use default value [%lf].",
        kCutPointCloudAngle);
    cut_point_cloud_angle_ = kCutPointCloudAngle;
  }

  // Duration to Retain Past Point Cloud
  double point_cloud_keep_period = 0.0;
  GetOptionalParam(shared_from_this(), "point_cloud_keep_period", point_cloud_keep_period, kPointCloudKeepPeriod);
  // Specify Default Value if Parameters are Invalid
  if (point_cloud_keep_period <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
        "Parameter 'point_cloud_keep_period' is invalid. Use default value [%lf].", kPointCloudKeepPeriod);
    point_cloud_keep_period = kPointCloudKeepPeriod;
  }
  // Set Parameters to Point Cloud Accumulation Class
  accumulator_->set_point_cloud_keep_period(point_cloud_keep_period);

  // Radius of Distance from Robot Center to Exclude from Acquired Point Cloud
  GetOptionalParam(shared_from_this(), "trimming_data_radius", trimming_data_radius_, kTrimmingDataRadius);
  // Specify Default Value if Parameters are Invalid
  if (trimming_data_radius_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
        "Parameter 'trimming_data_radius' is invalid. Use default value [%lf].", kTrimmingDataRadius);
    trimming_data_radius_ = kTrimmingDataRadius;
  }

  // Noise Filter Parameter (radius_search)
  GetOptionalParam(shared_from_this(), "noise_filter_radius_search", noise_filter_radius_search_,
      kNoiseFilterRadiusSearch);
  // Specify Default Value if Parameters are Invalid
  if (noise_filter_radius_search_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
        "Parameter 'noise_filter_radius_search' is invalid. Use default value [%lf].",
        kNoiseFilterRadiusSearch);
    noise_filter_radius_search_ = kNoiseFilterRadiusSearch;
  }

  // Noise Filter Parameter (neighbors)
  GetOptionalParam(shared_from_this(), "noise_filter_neighbors", noise_filter_neighbors_, kNoiseFilterNeighbors);
  if (noise_filter_neighbors_ <= 0) {
    RCLCPP_WARN(this->get_logger(),
        "Parameter 'noise_filter_neighbors' is invalid. Use default value [%d].",
        kNoiseFilterNeighbors);
    noise_filter_neighbors_ = kNoiseFilterNeighbors;
  }
}


// Point Cloud Callback Function
void PointCloudAccumulatorNode::PointCloudCallback(const PointCloud2Ptr msg) {
  if (!msg->data.empty()) {
    subscribed_cloud_->data.clear();
    *subscribed_cloud_ = *msg;
  }
}


// Service Callback to Start Point Cloud Accumulation
void PointCloudAccumulatorNode::StartAccumulateServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
                                                               std_srvs::srv::Empty::Response::SharedPtr res) {
  enable_function_ = true;
}


// Service Callback to Stop Point Cloud Accumulation
void PointCloudAccumulatorNode::StopAccumulateServiceCallback(std_srvs::srv::Empty::Request::SharedPtr req,
                                                              std_srvs::srv::Empty::Response::SharedPtr res) {
  // Discard Accumulated Point Cloud
  accumulator_->ClearPointCloud();
  enable_function_ = false;
}


// Merge Current Point Cloud with Accumulated Point Cloud
PointCloudPtr PointCloudAccumulatorNode::MergePointCloudInScanArea(const PointCloudPtr& input_cloud) {
  // Remove Point Cloud Within Sensor's Field of View
  PointCloudPtr output_cloud(new PointCloud());
  output_cloud->header.frame_id = input_cloud->header.frame_id;
  if (!input_cloud->points.empty()) {
    Eigen::Vector3d point_position = Eigen::Vector3d::Zero();
    for (PointCloud::iterator it = input_cloud->points.begin(); it != input_cloud->points.end(); ++it) {
      point_position << it->x, it->y, it->z;

      if ((point_position.norm() < max_saved_area_radius_) && (point_position.norm() > min_saved_area_radius_)) {
        // Do Not Merge Points Within Front Field of View
        double scan_angle = cut_point_cloud_angle_ * M_PI / 180.0;
        if ((it->x < 0.0) || (fabs(it->y) > fabs(tan(scan_angle) * it->x))) {
          output_cloud->points.push_back(*it);
        }
      }
    }
  }
  // Merge Accumulated Point Cloud with Current Point Cloud
  *output_cloud += *current_cloud_;
  return output_cloud;
}


// Publish Point Cloud
void PointCloudAccumulatorNode::PublishPointCloud(const PointCloudPtr& cloud) {
  // Convert Output Point Cloud to PointCloud2 Type
  if (!cloud->points.empty()) {
    PointCloud2Ptr pub_cloud(new PointCloud2());
    pcl::toROSMsg(*cloud, *pub_cloud);
    // Topic Publishing
    obstacle_point_cloud_publisher_->publish(*pub_cloud);
  }
}


// Save Point Cloud
void PointCloudAccumulatorNode::SavePointCloud(const PointCloudPtr& cloud) {
  // Clear Current Point Cloud
  current_cloud_->points.clear();
  // Save If Input Point Cloud is Not Empty
  if (!cloud->points.empty()) {
    // Convert Data Obtained in Sensor Reference Frame to Robot Base Coordinate System
    PointCloudPtr base_cloud = TransformPointCloud(cloud, base_frame_);
    // Among Data Obtained in Robot Base Coordinate System,
    // ・Point Cloud Close to Horizontal Distance from Origin
    // ・Point Cloud Outside Certain Height Range
    // Remove and Convert to Point Cloud with Height Set to 0
    PointCloudPtr reduced_cloud(new PointCloud());
    if (!base_cloud->points.empty()) {
      double distance_from_base = 0.0;
      for (PointCloud::iterator it = base_cloud->points.begin(); it != base_cloud->points.end(); ++it) {
        if ((!(isnan(it->x))) && (!(isnan(it->y))) && (!(isnan(it->z)))) {
          distance_from_base = it->x * it->x + it->y * it->y;
          if ((distance_from_base > trimming_data_radius_ * trimming_data_radius_) && (it->z < top_of_valid_space_) &&
              (it->z > bottom_of_valid_space_)) {
            pcl::PointXYZ point(it->x, it->y, 0.0);
            reduced_cloud->points.push_back(point);
          }
        }
      }
    }
    // Apply Downsampling to Point Cloud Projected to Height 0
    // Eliminate Duplicates
    PointCloudPtr down_sampled_cloud =
        PointCloudDownSampling(reduced_cloud, voxel_leaf_size_, noise_filter_radius_search_, noise_filter_neighbors_);
    // Save Point Cloud in Robot Base Coordinate System
    current_cloud_ = down_sampled_cloud;

    // Convert Data Obtained in Robot Base Coordinate System to Odometry Coordinate System
    if (!down_sampled_cloud->empty()) {
      down_sampled_cloud->header.frame_id = base_frame_;
      down_sampled_cloud->header.stamp = base_cloud->header.stamp;
      // Coordinate Transformation
      PointCloudPtr map_cloud = TransformPointCloud(down_sampled_cloud, odom_frame_);

      // Accumulate Point Cloud in Odometry Reference Coordinate System
      accumulator_->Accumulate(map_cloud);
    }
  }
}

// Main Processing
void PointCloudAccumulatorNode::NodeActionTimerCallback() {
  // If Function is Enabled, Merge Accumulated Point Cloud with
  // Current Point Cloud
  PointCloudPtr merged_cloud(new PointCloud());
  if (enable_function_ && !(subscribed_cloud_->data.empty())) {
    // Convert from PointCloud2 to pcl::PointXYZ
    PointCloudPtr sensor_cloud(new PointCloud());
    sensor_cloud->points.clear();
    // Convert to pcl Format
    pcl::fromROSMsg(*subscribed_cloud_, *sensor_cloud);
    // Save Point Cloud and Perform Accumulation Processing
    SavePointCloud(sensor_cloud);
    // Retrieve Accumulated Point Cloud and Perform Downsampling
    PointCloudPtr accumulated_cloud = PointCloudDownSampling(
        accumulator_->GetPointCloud(), voxel_leaf_size_,
        noise_filter_radius_search_, noise_filter_neighbors_);
    accumulated_cloud->header.frame_id = odom_frame_;
    accumulated_cloud->header.stamp = sensor_cloud->header.stamp;

    // Restore Point Cloud to Robot Base Coordinate System
    PointCloudPtr odom_cloud = TransformPointCloud(accumulated_cloud, base_frame_);
    // Merge Data from Measurement Area in Front of Robot
    merged_cloud = MergePointCloudInScanArea(odom_cloud);

    // Convert Point Cloud to Reference Coordinate System
    merged_cloud->header.frame_id = base_frame_;
    PointCloudPtr output_cloud = TransformPointCloud(merged_cloud, map_frame_);

    // Publish Point Cloud
    PublishPointCloud(output_cloud);
  }
}

// Perform Coordinate Transformation of Point Cloud to Specified Frame Reference
PointCloudPtr PointCloudAccumulatorNode::TransformPointCloud(const PointCloudPtr& input_cloud,
                                                             const std::string& base_frame) {
  PointCloudPtr transformed_cloud(new PointCloud());
  transformed_cloud->header.frame_id = base_frame;
  transformed_cloud->header.stamp = input_cloud->header.stamp;
  auto clock = rclcpp::Clock(RCL_ROS_TIME);
  if (!input_cloud->empty() && (pcl_conversions::fromPCL(input_cloud->header).stamp.sec != 0)) {
    try {
      const bool transform_is_found = tf_buffer_->canTransform(
          base_frame, input_cloud->header.frame_id,
          rclcpp::Time(pcl_conversions::fromPCL(input_cloud->header).stamp),
          rclcpp::Duration::from_seconds(kTfWaitDuration));
      if (transform_is_found) {
        bool ret = pcl_ros::transformPointCloud(base_frame, *input_cloud, *transformed_cloud, *tf_buffer_);
        if (!ret) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(),
              clock, kConsoleMessageIndicatePeriod,
              "transform error : [%s]->[%s]",
              input_cloud->header.frame_id.c_str(), base_frame.c_str());
        }
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), clock, kConsoleMessageIndicatePeriod,
              "wait for transform timeout...[%s]->[%s]",
              input_cloud->header.frame_id.c_str(), base_frame.c_str());
      }
    } catch (const std::exception& ex) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, kConsoleMessageIndicatePeriod,
          "transform error : %s", ex.what());
    }
  }
  return transformed_cloud;
}
}  // namespace tmc_point_cloud_accumulator
