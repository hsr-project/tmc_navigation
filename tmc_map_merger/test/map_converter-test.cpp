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
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "../src/map_converter.hpp"
#include "../src/map_merger.hpp"
#include "test_utils.hpp"

namespace tmc_map_merger {

// Generate a Map.
Map CreateBaseMap(const uint32_t width,
                  const uint32_t height,
                  const geometry_msgs::msg::Point& position) {
  Map map;
  map.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  map.header.frame_id = "map";
  map.info.resolution = 0.1;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = 0.0;
  map.info.origin.orientation.w = 1.0;
  map.info.origin.position.x = position.x;
  map.info.origin.position.y = position.y;
  map.info.origin.position.z = position.z;
  map.info.width = width;
  map.info.height = height;
  for (int idx = 0; idx < (width * height); ++idx) {
    map.data.push_back(kUnknown);
  }
  return map;
}

// Create Filters
std::vector<PointCloudFilter::Ptr> CreateFilters() {
  // Use empty Filters in the Converter TEST
  std::vector<PointCloudFilter::Ptr> out;
  return out;
}

// TEST if the result of GetOrigin with OccupancyGrid type is as expected
TEST(MapConverterTest, OccupancyGridGetOrigin) {
  // Setup
  geometry_msgs::msg::Point position;
  nav_msgs::msg::OccupancyGrid msg = CreateBaseMap(3, 3, position);;
  MapConverter<nav_msgs::msg::OccupancyGrid> converter;

  // Excersise
  geometry_msgs::msg::PoseStamped pose = converter.GetOrigin(msg);

  // Verify
  // Check if the obtained result matches the content of the msg
  ASSERT_EQ(msg.header.stamp.sec, pose.header.stamp.sec);
  ASSERT_EQ(msg.header.stamp.nanosec, pose.header.stamp.nanosec);
  ASSERT_EQ(msg.info.origin.position.x, pose.pose.position.x);
  ASSERT_EQ(msg.info.origin.position.y, pose.pose.position.y);
  ASSERT_EQ(msg.info.origin.position.z, pose.pose.position.z);
  ASSERT_EQ(msg.info.origin.orientation.x, pose.pose.orientation.x);
  ASSERT_EQ(msg.info.origin.orientation.y, pose.pose.orientation.y);
  ASSERT_EQ(msg.info.origin.orientation.z, pose.pose.orientation.z);
  ASSERT_EQ(msg.info.origin.orientation.w, pose.pose.orientation.w);
}

// TEST if the Map is updated when OccupancyGrid is converted.
// Do not check if the reflected content is correct as it depends on the Projector.
// Only check if the Map is updated.
TEST(MapConverterTest, OccupancyGridConvert) {
  // Setup
  geometry_msgs::msg::Point position;
  Map map;
  // Generate pre-conversion Map 5*5
  map = CreateBaseMap(5, 5, position);
  const std::vector<int8_t> map_data_init = map.data;
  // Generate conversion target
  position.x = 0.15;
  position.y = 0.25;
  nav_msgs::msg::OccupancyGrid msg = CreateBaseMap(3, 3, position);
  for (int idx = 0; idx < (msg.info.width * msg.info.height); ++idx) {
    msg.data[idx] = idx;
  }
  MapConverter<nav_msgs::msg::OccupancyGrid> converter;

  // Excersise
  converter.Convert(msg.info.origin, msg, MapOperator<UpdateIfGreaterMapAdapter>(map));

  // Verify
  ASSERT_NE(map_data_init, map.data);
}

class MapConverterTestFixture : public testing::Test {
 protected:
  virtual void SetUp() {
    test_node_ = CreateParameterNode("map_converter-test.yaml");
  }
  std::shared_ptr<rclcpp::Node> test_node_;
};


// TEST if the result of converting PointCloud is reflected in the Map.
// Check if the Map is updated.
// Do not check the validity of the content as it depends on the drawer
TEST_F(MapConverterTestFixture, PointCloudConvert) {
  // Setup
  const std::string input_name = "rgbd_sensor";
  geometry_msgs::msg::Point position;
  Map map;
  // Generate pre-conversion Map 20*20
  map = CreateBaseMap(20, 20, position);
  const std::vector<int8_t> map_data_init = map.data;
  // Generate conversion target
  PointCloud msg;
  msg.header.frame_id = "map";
  msg.height = 1;
  msg.width = 1;
  msg.is_dense = true;
  msg.points.push_back(pcl::PointXYZ(1.0, 1.0, 0.0));
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.1;
  pose.position.y = 0.1;
  pose.position.z = 0.1;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.00;
  MapConverter<PointCloud> converter(test_node_, input_name, CreateFilters());

  // Excersise
  converter.Convert(pose, msg, MapOperator<UpdateIfGreaterMapAdapter>(map));

  // Verify
  ASSERT_NE(map_data_init, map.data);
}

// TEST if the result of GetOrigin with LaserScan type is as expected
TEST_F(MapConverterTestFixture, LaserScanGetOrigin) {
  // Setup
  const std::string input_name = "laser_scan";
  sensor_msgs::msg::LaserScan msg;
  const int data_num = 100;
  msg.angle_max = 1.0;
  msg.angle_min = -msg.angle_max;
  msg.angle_increment = (msg.angle_max - msg.angle_min) / static_cast<float>(data_num);
  msg.range_max = 0.5;
  msg.range_min = 0.0;
  for (int i = 0; i < data_num; ++i) {
    // 0 to 1.0 [m]
    const float range = msg.range_max * i / static_cast<float>(data_num);
    msg.ranges.push_back(range);
  }
  msg.header.frame_id = "map";
  msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  MapConverter<sensor_msgs::msg::LaserScan> converter(test_node_, input_name, CreateFilters());

  // Excersise
  geometry_msgs::msg::PoseStamped pose = converter.GetOrigin(msg);

  // Verify
  // Check if the obtained result matches the content of the msg
  ASSERT_EQ(msg.header.stamp.sec, pose.header.stamp.sec);
  ASSERT_EQ(msg.header.stamp.nanosec, pose.header.stamp.nanosec);
  ASSERT_EQ(0.0, pose.pose.position.x);
  ASSERT_EQ(0.0, pose.pose.position.y);
  ASSERT_EQ(0.0, pose.pose.position.z);
  ASSERT_EQ(0.0, pose.pose.orientation.x);
  ASSERT_EQ(0.0, pose.pose.orientation.y);
  ASSERT_EQ(0.0, pose.pose.orientation.z);
  ASSERT_EQ(1.0, pose.pose.orientation.w);
}

// TEST the conversion of LaserScan type.
// Check if the Map is updated.
// Do not check the validity of the content as it depends on the PointCloud Converter
TEST_F(MapConverterTestFixture, LaseScanConvert) {
  // Setup
  const std::string input_name = "laser_scan";
  geometry_msgs::msg::Point position;
  Map map;
  // Generate pre-conversion Map 20*20
  map = CreateBaseMap(20, 20, position);
  const std::vector<int8_t> map_data_init = map.data;
  // Generate conversion target
  sensor_msgs::msg::LaserScan msg;
  const int data_num = 100;
  msg.angle_max = 1.0;
  msg.angle_min = -msg.angle_max;
  msg.angle_increment = (msg.angle_max - msg.angle_min) / static_cast<float>(data_num);
  msg.range_max = 0.5;
  msg.range_min = 0.0;
  for (int i = 0; i < data_num; ++i) {
    // 0 to 1.0 [m]
    const float range = msg.range_max * i / static_cast<float>(data_num);
    msg.ranges.push_back(range);
  }
  msg.header.frame_id = "map";
  msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.9;
  pose.position.y = 1.9;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  MapConverter<sensor_msgs::msg::LaserScan> converter(test_node_, input_name, CreateFilters());

  // Excersise
  converter.Convert(pose, msg, MapOperator<UpdateIfGreaterMapAdapter>(map));

  // Verify
  ASSERT_NE(map_data_init, map.data);
}

// TEST if the result of GetOrigin with PointCloud2 type is as expected
TEST_F(MapConverterTestFixture, PointCloud2GetOrigin) {
  // Setup
  const std::string input_name = "rgbd_sensor";
  PointCloud pcl_object;
  pcl_object.header.frame_id = "map";
  pcl_object.height = 1;
  pcl_object.width = 1;
  pcl_object.is_dense = false;
  pcl_object.points.push_back(pcl::PointXYZ(1.0, 1.0, 0.0));
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pcl_object, msg);
  MapConverter<sensor_msgs::msg::PointCloud2> converter(test_node_, input_name, CreateFilters());

  // Excersise
  geometry_msgs::msg::PoseStamped pose = converter.GetOrigin(msg);

  // Verify
  // Check if the obtained result matches the content of the msg
  ASSERT_EQ(msg.header.stamp.sec, pose.header.stamp.sec);
  ASSERT_EQ(msg.header.stamp.nanosec, pose.header.stamp.nanosec);
  ASSERT_EQ(0.0, pose.pose.position.x);
  ASSERT_EQ(0.0, pose.pose.position.y);
  ASSERT_EQ(0.0, pose.pose.position.z);
  ASSERT_EQ(0.0, pose.pose.orientation.x);
  ASSERT_EQ(0.0, pose.pose.orientation.y);
  ASSERT_EQ(0.0, pose.pose.orientation.z);
  ASSERT_EQ(1.0, pose.pose.orientation.w);
}

// TEST the conversion of PointCloud2 type
// Check if the Map is updated.
// Do not check the validity of the content as it depends on the PointCloud Converter
TEST_F(MapConverterTestFixture, PointCloud2Convert) {
  // Setup
  const std::string input_name = "rgbd_sensor";
  geometry_msgs::msg::Point position;
  Map map;
  // Generate pre-conversion Map 20*20
  map = CreateBaseMap(20, 20, position);
  const std::vector<int8_t> map_data_init = map.data;

  // Generate conversion target
  PointCloud pcl_object;
  pcl_object.header.frame_id = "map";
  pcl_object.height = 1;
  pcl_object.width = 1;
  pcl_object.is_dense = true;
  pcl_object.points.push_back(pcl::PointXYZ(1.0, 1.0, 0.0));
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pcl_object, msg);

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.1;
  pose.position.y = 0.1;
  pose.position.z = 0.1;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.00;
  MapConverter<sensor_msgs::msg::PointCloud2> converter(test_node_, input_name, CreateFilters());

  // Excersise
  converter.Convert(pose, msg, MapOperator<UpdateIfGreaterMapAdapter>(map));

  // Verify
  ASSERT_NE(map_data_init, map.data);
}

}  // namespace tmc_map_merger

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
