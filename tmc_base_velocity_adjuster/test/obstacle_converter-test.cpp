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
#include "../src/obstacle_converter.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {
// Parameters for normal operation tests
constexpr double kFilterLeafSize = 0.1;
constexpr double kFilterAreaRadius = 100.0;
// Set the obstacle cloud frame to be the same as the cart frame to omit tf publishing
constexpr const char* const kObstacleFrame = "base_link";
// Minimal interval [m] for voxel filter testing
constexpr double kSmallDistance = 0.001;
}

namespace tmc_base_velocity_adjuster {

class TestNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options) {}

  void SetFilterLeafSizeParam(const double leaf_size) {
    rclcpp::Parameter filter_leaf_size_param(
        "obstacle_converter.filter_leaf_size", rclcpp::ParameterValue(leaf_size));
    this->declare_parameter(filter_leaf_size_param.get_name(), filter_leaf_size_param.get_type());
    this->set_parameter(filter_leaf_size_param);
  }

  void SetFilterAreaRadiusParam(const double area_radius) {
    rclcpp::Parameter filter_area_radius(
        "obstacle_converter.filter_area_radius", rclcpp::ParameterValue(area_radius));
    this->declare_parameter(filter_area_radius.get_name(), filter_area_radius.get_type());
    this->set_parameter(filter_area_radius);
  }
};

/// ObstacleConverter test fixture
class ObstacleConverterTest : public ::testing::Test {
 public:
  ObstacleConverterTest() {}

  virtual void SetUp() {
    rclcpp::NodeOptions option;
    option.allow_undeclared_parameters();
    option.automatically_declare_parameters_from_overrides(true);
    test_node_ = std::make_shared<TestNode>(option);
  }

 protected:
  std::shared_ptr<TestNode> test_node_;
};

/// ObstacleConverter test
/// Points within the same voxel are filtered into one point
TEST_F(ObstacleConverterTest, FilterPointsInTheSameVoxel) {
  // setup
  // Parameter settings
  test_node_->SetFilterLeafSizeParam(kFilterLeafSize);
  test_node_->SetFilterAreaRadiusParam(kFilterAreaRadius);
  // Generate the test target instance
  ObstacleConverter<sensor_msgs::msg::PointCloud2, pcl::PointCloud<pcl::PointXYZ>> converter(test_node_);
  // Set input point cloud: place 2 points within the same voxel and 1 point crossing the voxel boundary
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  input_cloud.points.push_back(pcl::PointXYZ(kFilterLeafSize - kSmallDistance, 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(kFilterLeafSize - kSmallDistance * 2.0, 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(kFilterLeafSize + kSmallDistance, 0.0, 0.0));
  // Convert to PontCloud2
  sensor_msgs::msg::PointCloud2::Ptr input_cloud2(new sensor_msgs::msg::PointCloud2());
  pcl::toROSMsg(input_cloud, *input_cloud2);
  input_cloud2->header.frame_id = kObstacleFrame;
  input_cloud2->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  // exercise
  // Execute converter
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  converter.Convert(input_cloud2, output_cloud);

  // verify
  // Confirm that points within the same voxel are filtered to 2 points
  ASSERT_EQ(2, output_cloud->points.size());
  // One of the points crosses the voxel boundary
  EXPECT_TRUE((output_cloud->points[0].x > kFilterLeafSize) ||
              (output_cloud->points[1].x > kFilterLeafSize));
}

/// ObstacleConverter test
/// Points outside the specified distance are filtered
TEST_F(ObstacleConverterTest, FilterPointsOutOfArea) {
  // setup
  // Parameter settings
  test_node_->SetFilterLeafSizeParam(kFilterLeafSize);
  test_node_->SetFilterAreaRadiusParam(kFilterAreaRadius);
  // Generate the test target instance
  ObstacleConverter<sensor_msgs::msg::PointCloud2, pcl::PointCloud<pcl::PointXYZ>> converter(test_node_);

  // Set input point cloud: place 1 point inside and 1 point outside the range for both X and Y
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  input_cloud.points.push_back(pcl::PointXYZ(kFilterAreaRadius - kFilterLeafSize, 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(kFilterAreaRadius + kFilterLeafSize, 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(-(kFilterAreaRadius - kFilterLeafSize), 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(-(kFilterAreaRadius + kFilterLeafSize), 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(0.0, kFilterAreaRadius - kFilterLeafSize, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(0.0, kFilterAreaRadius + kFilterLeafSize, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(0.0, -(kFilterAreaRadius - kFilterLeafSize), 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(0.0, -(kFilterAreaRadius + kFilterLeafSize), 0.0));
  // Convert to PontCloud2
  sensor_msgs::msg::PointCloud2::Ptr input_cloud2(new sensor_msgs::msg::PointCloud2());
  pcl::toROSMsg(input_cloud, *input_cloud2);
  input_cloud2->header.frame_id = kObstacleFrame;
  input_cloud2->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  // exercise
  // Execute converter
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  converter.Convert(input_cloud2, output_cloud);

  // verify
  // Confirm the number of points in the filtered point cloud
  ASSERT_EQ(4, output_cloud->points.size());
  // Confirm that all remaining points are within the specified distance
  for (const auto& it : output_cloud->points) {
    const Eigen::Vector3d point_pos(it.x, it.y, 0.0);
    EXPECT_LT(point_pos.norm(), kFilterAreaRadius);
  }
}

/// ObstacleConverter test
/// Default values are used if rosparam settings are not set
TEST_F(ObstacleConverterTest, NoParametersSet) {
  // setup
  // Generate instance without parameter settings
  ObstacleConverter<sensor_msgs::msg::PointCloud2, pcl::PointCloud<pcl::PointXYZ>> converter(test_node_);
  // Set input point cloud: place 2 points within the same voxel, 1 point crossing the voxel boundary, and 1 point before and after the area boundary, totaling 5 points
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  input_cloud.points.push_back(pcl::PointXYZ(kFilterLeafSizeDefault - kSmallDistance, 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(kFilterLeafSizeDefault - kSmallDistance * 2.0, 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(kFilterLeafSizeDefault + kSmallDistance, 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(kFilterAreaDefault - kFilterLeafSizeDefault, 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(kFilterAreaDefault + kFilterLeafSizeDefault, 0.0, 0.0));
  // Convert to PontCloud2
  sensor_msgs::msg::PointCloud2::Ptr input_cloud2(new sensor_msgs::msg::PointCloud2());
  pcl::toROSMsg(input_cloud, *input_cloud2);
  input_cloud2->header.frame_id = kObstacleFrame;
  input_cloud2->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  // exercise
  // Execute converter
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  converter.Convert(input_cloud2, output_cloud);

  // verify
  // Confirm that 1 point is removed by the voxel filter and 1 point by the range filter
  ASSERT_EQ(3, output_cloud->points.size());
  // Since the voxel filter itself is out of scope, coordinate verification is only performed in normal operation tests
  // Confirm that the range filter is performed with default values
  for (const auto& it : output_cloud->points) {
    const Eigen::Vector3d point_pos(it.x, it.y, 0.0);
    EXPECT_LT(point_pos.norm(), kFilterAreaDefault);
  }
}

/// ObstacleConverter test
/// Default values are used if parameter settings are invalid
TEST_F(ObstacleConverterTest, InvalidParametersSet) {
  // setup
  // Invalid parameter settings
  test_node_->SetFilterLeafSizeParam(-1.0);
  test_node_->SetFilterAreaRadiusParam(-1.0);
  ObstacleConverter<sensor_msgs::msg::PointCloud2, pcl::PointCloud<pcl::PointXYZ>> converter(test_node_);
  // Set input point cloud: place 2 points within the same voxel, 1 point crossing the voxel boundary, and 1 point before and after the area boundary, totaling 5 points
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  input_cloud.points.push_back(pcl::PointXYZ(kFilterLeafSizeDefault - kSmallDistance, 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(kFilterLeafSizeDefault - kSmallDistance * 2.0, 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(kFilterLeafSizeDefault + kSmallDistance, 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(kFilterAreaDefault - kFilterLeafSizeDefault, 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(kFilterAreaDefault + kFilterLeafSizeDefault, 0.0, 0.0));
  // Convert to PontCloud2
  sensor_msgs::msg::PointCloud2::Ptr input_cloud2(new sensor_msgs::msg::PointCloud2());
  pcl::toROSMsg(input_cloud, *input_cloud2);
  input_cloud2->header.frame_id = kObstacleFrame;
  input_cloud2->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  // exercise
  // Execute converter
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  converter.Convert(input_cloud2, output_cloud);

  // verify
  // Confirm that 1 point is removed by the voxel filter and 1 point by the range filter
  ASSERT_EQ(3, output_cloud->points.size());
  // Since the voxel filter itself is out of scope, coordinate verification is only performed in normal operation tests
  // Confirm that the range filter is performed with default values
  for (const auto& it : output_cloud->points) {
    const Eigen::Vector3d point_pos(it.x, it.y, 0.0);
    EXPECT_LT(point_pos.norm(), kFilterAreaDefault);
  }
}
}  // namespace tmc_base_velocity_adjuster

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
