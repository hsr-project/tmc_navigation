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
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// TODO(syuuhei_shiro) tmc_rostest_utilsのROS2化
// #include <tmc_rostest_utils/util_function.hpp>
#include "../src/obstacle_input.hpp"
#include "test_utils.hpp"

namespace {
// Parameters for normal test cases
constexpr double kFilterLeafSize = 0.1;
constexpr double kFilterAreaRadius = 100.0;
// Set the obstacle cloud frame to be the same as the cart frame to omit tf publishing
constexpr const char* const kObstacleFrame = "base_link";
// Subscriber timeout [s]
constexpr double kTimeOut = 1.0;
}  // anonymous namespace

namespace tmc_base_velocity_adjuster {

/// ObstacleInput test
/// Verify that obstacle data is retained through subscription
TEST(ObstacleInputTest, GetObstacleData) {
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("test_node", option);
  // Initialize ROS interface
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle", 1);
  rclcpp::Rate rate(10.0);

  // Set parameters
  rclcpp::Parameter filter_leaf_size_param(
      "obstacle_converter.filter_leaf_size", rclcpp::ParameterValue(kFilterLeafSize));
  node->declare_parameter(filter_leaf_size_param.get_name(), filter_leaf_size_param.get_type());
  node->set_parameter(filter_leaf_size_param);
  rclcpp::Parameter filter_area_radius_param(
      "obstacle_converter.filter_area_radius", rclcpp::ParameterValue(kFilterAreaRadius));
  node->declare_parameter(filter_area_radius_param.get_name(), filter_area_radius_param.get_type());
  node->set_parameter(filter_area_radius_param);

  // Configure input point cloud: Set two points within the unfiltered range (filter validity is not checked in this test)
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  input_cloud.points.push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
  input_cloud.points.push_back(pcl::PointXYZ(kFilterAreaRadius - kFilterLeafSize, 0.0, 0.0));
  // Convert to PointCloud2
  sensor_msgs::msg::PointCloud2 input_cloud2;
  pcl::toROSMsg(input_cloud, input_cloud2);
  input_cloud2.header.frame_id = kObstacleFrame;
  input_cloud2.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  // Generate the test target instance
  ObstacleInput<sensor_msgs::msg::PointCloud2, pcl::PointCloud<pcl::PointXYZ>> obstacle_input(node);
  // Wait for subscriber to start
  EXPECT_TRUE(WaitUntil(node,
      [&]() { return pub_obstacle->get_subscription_count() > 0; },
      kTimeOut));
  // Publish point cloud
  pub_obstacle->publish(input_cloud2);
  // Wait for subscription
  EXPECT_TRUE(WaitUntil(node,
      [&]() { return obstacle_input.GetObstacle()->points.size() > 0; },
      kTimeOut));

  // Retrieve subscription results
  pcl::PointCloud<pcl::PointXYZ>::Ptr subscribed_cloud;
  subscribed_cloud = obstacle_input.GetObstacle();

  // verify
  // Verify that the number of points matches the published data
  EXPECT_EQ(input_cloud.points.size(), subscribed_cloud->points.size());
}

}  // namespace tmc_base_velocity_adjuster

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
