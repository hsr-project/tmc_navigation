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
/// @file     grid_map_server_node_test.cpp
/// @brief    Map loading test
/// @author   Copyright (C) 2013 TOYOTA MOTOR CORPRATION.
#include <fstream>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/optional.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/parameter_map.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <tmc_navigation_msgs/msg/occupancy_grid_uint.hpp>
#include <tmc_navigation_msgs/srv/reload_map.hpp>

#include "../src/tmc_grid_map_server/grid_map_server_node.hpp"
#include "../src/tmc_grid_map_server/param.hpp"

namespace {
constexpr float kMapOriginZ = 0.0;
constexpr const char* const kFrameId = "map";
/// Map update wait timeout[s]
constexpr double kWaitForMapTimeOut = 10.0;
/// Map update wait cycle[Hz]
constexpr double kWaitForMapRate = 10.0;

geometry_msgs::msg::Quaternion CreateQuaternionMsgFromYaw(const double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

// TODO(syuuhei_shiro): tmc_rostest_utilをROS2化してそこに置く
// Load parameters from yaml file
void LoadParameterFromYaml(std::shared_ptr<rclcpp::Node> node,
    const std::string& yaml_directory, const std::string& yaml_name) {
  const std::string yaml_path = yaml_directory + yaml_name;
  // Load yaml and generate ParameterMap
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_params_t* yaml_params = rcl_yaml_node_struct_init(allocator);
  rcl_parse_yaml_file(yaml_path.c_str(), yaml_params);
  rclcpp::ParameterMap yaml_param_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);
  // Set ros parameters to node
  const std::string parameter_space = "/" + std::string(node->get_name());
  auto iter = yaml_param_map.find(parameter_space);
  for (auto& param : iter->second) {
    node->declare_parameter(param.get_name(), param.get_type());
    node->set_parameter(param);
  }
}
}  // namespace

namespace tmc_grid_map_server {
using std::placeholders::_1;

/// Test node
class TestNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("grid_map_server_test_node", options) {}

  /// Initialization
  void Init() {
    // Read test parameters
    // Map setting file name
    std::string map_yaml;
    EXPECT_TRUE(GetParam(shared_from_this(), "map_yaml", map_yaml));
    // Map setting file name that reverses distance_map and obstacle_map from map_yaml
    EXPECT_TRUE(GetParam(shared_from_this(), "reverse_map_yaml", reverse_map_yaml_));
    // Map width (number of grids)
    EXPECT_TRUE(GetParam(shared_from_this(), "map_width", map_width_));
    // Map height (number of grids)
    EXPECT_TRUE(GetParam(shared_from_this(), "map_height", map_height_));
    // Expected value of distance_map data
    EXPECT_TRUE(GetParam(shared_from_this(), "expect_distance_map_data", expect_distance_map_data_));
    // Expected value of obstacle_map data
    EXPECT_TRUE(GetParam(shared_from_this(), "expect_obstacle_map_data", expect_obstacle_map_data_));
    // Expected value of TMC type distance_map data
    EXPECT_TRUE(GetParam(shared_from_this(), "expect_tmc_distance_map_data", expect_tmc_distance_map_data_));
    // Expected value of TMC type obstacle_map data
    EXPECT_TRUE(GetParam(shared_from_this(), "expect_tmc_obstacle_map_data", expect_tmc_obstacle_map_data_));
    // Data issued when updating the map
    EXPECT_TRUE(GetParam(shared_from_this(), "update_map_data", update_map_data_));
    // Expected value of TMC type distance_map and TMC type obstacle_map data after map update
    EXPECT_TRUE(GetParam(shared_from_this(), "expect_update_tmc_map_data", expect_update_tmc_map_data_));

    // Read map_yaml
    const std::string map_yaml_path = ament_index_cpp::get_package_share_directory("tmc_grid_map_server") +
        "/test/maps/" + map_yaml;
    std::ifstream ifs_map_yaml(map_yaml_path.c_str());
    if (ifs_map_yaml.fail()) {
      throw std::runtime_error("Can not open " + map_yaml_path);
    }
    YAML::Node map_yaml_node = YAML::Load(ifs_map_yaml);
    // Map resolution
    map_resolution_ = map_yaml_node["resolution"].as<double>();
    YAML::Node map_origin = map_yaml_node["origin"];
    // Map origin
    map_origin_x_ = map_origin[0].as<double>();
    map_origin_y_ = map_origin[1].as<double>();
    map_origin_yaw_ = map_origin[2].as<double>();

    ResetSubscribeMaps();
    // Pub, Sub definitions
    sub_distance_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "static_distance_ros_map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&TestNode::CallbackDistanceMap, this, _1));
    sub_obstacle_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "static_obstacle_ros_map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&TestNode::CallbackObstacleMap, this, _1));
    sub_tmc_distance_map_ = this->create_subscription<tmc_navigation_msgs::msg::OccupancyGridUint>(
        "static_distance_map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&TestNode::CallbackTMCDistanceMap, this, _1));
    sub_tmc_obstacle_map_ = this->create_subscription<tmc_navigation_msgs::msg::OccupancyGridUint>(
        "static_obstacle_map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&TestNode::CallbackTMCObstacleMap, this, _1));
    pub_online_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/update_map", 10);

    // Service definition
    srv_reload_map_ = this->create_client<tmc_navigation_msgs::srv::ReloadMap>("/reload_map");
  }
  // Check if Pub, Sub connections are established
  bool CheckConnection() {
    return sub_distance_map_->get_publisher_count() > 0 &&
           sub_obstacle_map_->get_publisher_count() > 0 &&
           sub_tmc_distance_map_->get_publisher_count() > 0 &&
           sub_tmc_obstacle_map_->get_publisher_count() > 0 &&
           pub_online_map_->get_subscription_count() > 0;
  }


  /// Publish map topic for online updates
  void PublishMapTopic() {
    nav_msgs::msg::OccupancyGrid map;
    map.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    map.header.frame_id = kFrameId;
    map.info.map_load_time = rclcpp::Clock(RCL_ROS_TIME).now();
    map.info.resolution = map_resolution_;
    map.info.width = map_width_;
    map.info.height = map_height_;
    map.info.origin.position.x = map_origin_x_;
    map.info.origin.position.y = map_origin_y_;
    map.info.origin.position.z = kMapOriginZ;
    map.info.origin.orientation = CreateQuaternionMsgFromYaw(map_origin_yaw_);
    map.data.resize(map_width_ * map_height_);
    for (int32_t i = 0; i < map_width_ * map_height_; i++) {
      map.data[i] = update_map_data_[i];
    }
    pub_online_map_->publish(map);
  }

  // Call map reload service
  // Read the configuration file that specifies the distance and obstacle images in reverse
  void ReloadReverseMap() {
    auto reload_map_request = std::make_shared<tmc_navigation_msgs::srv::ReloadMap::Request>();
    reload_map_request->new_map_yaml = ament_index_cpp::get_package_share_directory("tmc_grid_map_server") +
        "/test/maps/" + reverse_map_yaml_;
    srv_reload_map_->async_send_request(reload_map_request);
  }

  /// Initialization of subscription map
  void ResetSubscribeMaps() {
    distance_map_ = boost::none;
    obstacle_map_ = boost::none;
    tmc_distance_map_ = boost::none;
    tmc_obstacle_map_ = boost::none;
  }

  // Accessor
  /// Distance map
  boost::optional<nav_msgs::msg::OccupancyGrid> distance_map() { return distance_map_; }
  /// Obstacle map
  boost::optional<nav_msgs::msg::OccupancyGrid> obstacle_map() { return obstacle_map_; }
  /// TMC type distance map
  boost::optional<tmc_navigation_msgs::msg::OccupancyGridUint> tmc_distance_map() { return tmc_distance_map_; }
  /// TMC type obstacle map
  boost::optional<tmc_navigation_msgs::msg::OccupancyGridUint> tmc_obstacle_map() { return tmc_obstacle_map_; }
  // Map resolution
  double map_resolution() { return map_resolution_; }
  // Map width (number of grids)
  int32_t map_width() { return map_width_; }
  // Map height (number of grids)
  int32_t map_height() { return map_height_; }
  // Map origin
  double map_origin_x() { return map_origin_x_; }
  double map_origin_y() { return map_origin_y_; }
  double map_origin_yaw() { return map_origin_yaw_; }

  // Expected value of distance_map data
  std::vector<int64_t> expect_distance_map_data() { return expect_distance_map_data_; }
  // Expected value of obstacle_map data
  std::vector<int64_t> expect_obstacle_map_data() { return expect_obstacle_map_data_; }
  // Expected value of TMC type distance_map data
  std::vector<int64_t> expect_tmc_distance_map_data() { return expect_tmc_distance_map_data_; }
  // Expected value of TMC type obstacle_map data
  std::vector<int64_t> expect_tmc_obstacle_map_data() { return expect_tmc_obstacle_map_data_; }
  // Data issued when updating the map
  std::vector<int64_t> update_map_data() { return update_map_data_; }
  // Expected value of TMC type distance_map and TMC type obstacle_map data after map update
  std::vector<int64_t> expect_update_tmc_map_data() { return expect_update_tmc_map_data_; }

 private:
  /// Distance map topic subscription callback
  void CallbackDistanceMap(const nav_msgs::msg::OccupancyGrid::SharedPtr distance_map) {
    distance_map_ = *distance_map;
  }
  /// Obstacle map topic subscription callback
  void CallbackObstacleMap(const nav_msgs::msg::OccupancyGrid::SharedPtr obstacle_map)  {
    obstacle_map_ = *obstacle_map;
  }
  /// TMC type distance map topic subscription callback
  void CallbackTMCDistanceMap(const tmc_navigation_msgs::msg::OccupancyGridUint::SharedPtr tmc_distance_map) {
    tmc_distance_map_ = *tmc_distance_map;
  }
  /// TMC type obstacle map topic subscription callback
  void CallbackTMCObstacleMap(const tmc_navigation_msgs::msg::OccupancyGridUint::SharedPtr tmc_obstacle_map) {
    tmc_obstacle_map_ = *tmc_obstacle_map;
  }

  /// Topic subscription (distance map)
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_distance_map_;
  /// Topic subscription (obstacle map)
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_obstacle_map_;
  /// Topic subscription (TMC type distance map)
  rclcpp::Subscription<tmc_navigation_msgs::msg::OccupancyGridUint>::SharedPtr sub_tmc_distance_map_;
  /// Topic subscription (TMC type obstacle map)
  rclcpp::Subscription<tmc_navigation_msgs::msg::OccupancyGridUint>::SharedPtr sub_tmc_obstacle_map_;
  /// Topic publishing (updated map)
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_online_map_;
  /// Service call (map reload)
  rclcpp::Client<tmc_navigation_msgs::srv::ReloadMap>::SharedPtr srv_reload_map_;
  /// Distance map
  boost::optional<nav_msgs::msg::OccupancyGrid> distance_map_;
  /// Obstacle map
  boost::optional<nav_msgs::msg::OccupancyGrid> obstacle_map_;
  /// TMC type distance map
  boost::optional<tmc_navigation_msgs::msg::OccupancyGridUint> tmc_distance_map_;
  /// TMC type obstacle map
  boost::optional<tmc_navigation_msgs::msg::OccupancyGridUint> tmc_obstacle_map_;
  /// Online updated map
  nav_msgs::msg::OccupancyGrid online_map_;
  // Map setting file name that reverses distance_map and obstacle_map from map_yaml
  std::string reverse_map_yaml_;
  // Map resolution
  double map_resolution_;
  // Map width (number of grids)
  int32_t map_width_;
  // Map height (number of grids)
  int32_t map_height_;
  // Map origin
  double map_origin_x_;
  double map_origin_y_;
  double map_origin_yaw_;
  // Expected value of distance_map data
  std::vector<int64_t> expect_distance_map_data_;
  // Expected value of obstacle_map data
  std::vector<int64_t> expect_obstacle_map_data_;
  // Expected value of TMC type distance_map data
  std::vector<int64_t> expect_tmc_distance_map_data_;
  // Expected value of TMC type obstacle_map data
  std::vector<int64_t> expect_tmc_obstacle_map_data_;
  // Data issued when updating the map
  std::vector<int64_t> update_map_data_;
  // Expected value of TMC type distance_map and TMC type obstacle_map data after map update
  std::vector<int64_t> expect_update_tmc_map_data_;
};

/// Class for node testing
class TestMapServer : public ::testing::Test {
 public:
  TestMapServer() {}
  virtual ~TestMapServer() {}

  virtual void SetUp() {
    std::string parameter_file = std::string(getenv("PARAMETER_FILE"));
    parameter_file.erase(0, 1);
    rclcpp::NodeOptions option;
    option.allow_undeclared_parameters();
    option.automatically_declare_parameters_from_overrides(true);
    const std::string yaml_directory = ament_index_cpp::get_package_share_directory("tmc_grid_map_server") +
        "/test/parameter/";
    // Test node generation
    test_node_ = std::make_shared<TestNode>(option);
    LoadParameterFromYaml(test_node_, yaml_directory, parameter_file);
    test_node_->Init();
    rclcpp::Rate rate(10.0);
    while (!test_node_->CheckConnection()) {
      rate.sleep();
      SpinOnce();
    }
  }
  virtual void TearDown() {}

 protected:
  void SpinOnce() {
    rclcpp::spin_some(test_node_);
  }

  /// Wait until the map is updated
  bool WaitForMapUpdate() {
    rclcpp::Rate rate(kWaitForMapRate);
    rclcpp::Time start = rclcpp::Clock(RCL_ROS_TIME).now();
    // Wait until the map is updated
    while (!(test_node_->distance_map() && test_node_->obstacle_map() &&
        test_node_->tmc_distance_map() && test_node_->tmc_obstacle_map())) {
      // False if wait time times out
      if (rclcpp::Clock(RCL_ROS_TIME).now() - start > rclcpp::Duration::from_seconds(kWaitForMapTimeOut)) {
        return false;
      }
      SpinOnce();
      rate.sleep();
    }
    // True if all maps are updated
    return true;
  }

  std::shared_ptr<GridMapServerNode> grid_map_server_node_;
  std::shared_ptr<TestNode> test_node_;
};

/// Test of map node
TEST_F(TestMapServer, Map) {
  // Spin until subscribing to the map
  ASSERT_TRUE(WaitForMapUpdate());
  const double map_resolution = test_node_->map_resolution();
  const int32_t map_width = test_node_->map_width();
  const int32_t map_height = test_node_->map_height();
  const double map_origin_x = test_node_->map_origin_x();
  const double map_origin_y = test_node_->map_origin_y();
  const double map_origin_yaw = test_node_->map_origin_yaw();
  // Test of distance_map
  const nav_msgs::msg::OccupancyGrid distance_map = test_node_->distance_map().get();
  // Test of meta data
  EXPECT_FLOAT_EQ(distance_map.info.resolution, map_resolution);
  EXPECT_EQ(distance_map.info.width, map_width);
  EXPECT_EQ(distance_map.info.height, map_height);
  EXPECT_FLOAT_EQ(distance_map.info.origin.position.x, map_origin_x);
  EXPECT_FLOAT_EQ(distance_map.info.origin.position.y, map_origin_y);
  EXPECT_FLOAT_EQ(distance_map.info.origin.position.z, kMapOriginZ);
  geometry_msgs::msg::Quaternion orientation;
  orientation = CreateQuaternionMsgFromYaw(map_origin_yaw);
  EXPECT_FLOAT_EQ(distance_map.info.origin.orientation.x, orientation.x);
  EXPECT_FLOAT_EQ(distance_map.info.origin.orientation.y, orientation.y);
  EXPECT_FLOAT_EQ(distance_map.info.origin.orientation.z, orientation.z);
  EXPECT_FLOAT_EQ(distance_map.info.origin.orientation.w, orientation.w);
  // Test of header
  EXPECT_STREQ(distance_map.header.frame_id.c_str(), kFrameId);
  // Test of data
  const std::vector<int64_t> expect_distance_map_data = test_node_->expect_distance_map_data();
  ASSERT_EQ(expect_distance_map_data.size(), distance_map.data.size());
  for (uint32_t i = 0; i < distance_map.info.width * distance_map.info.height; ++i) {
    EXPECT_EQ(distance_map.data[i], expect_distance_map_data[i]);
  }

  // Test of obstacle_map
  const nav_msgs::msg::OccupancyGrid obstacle_map = test_node_->obstacle_map().get();
  // Test of meta data
  EXPECT_FLOAT_EQ(obstacle_map.info.resolution, map_resolution);
  EXPECT_EQ(obstacle_map.info.width, map_width);
  EXPECT_EQ(obstacle_map.info.height, map_height);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.position.x, map_origin_x);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.position.y, map_origin_y);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.position.z, kMapOriginZ);
  orientation = CreateQuaternionMsgFromYaw(map_origin_yaw);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.orientation.x, orientation.x);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.orientation.y, orientation.y);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.orientation.z, orientation.z);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.orientation.w, orientation.w);
  // Test of header
  EXPECT_STREQ(obstacle_map.header.frame_id.c_str(), kFrameId);
  // Test of data
  const std::vector<int64_t> expect_obstacle_map_data = test_node_->expect_obstacle_map_data();
  ASSERT_EQ(expect_obstacle_map_data.size(), obstacle_map.data.size());
  for (uint32_t i = 0; i < obstacle_map.info.width * obstacle_map.info.height; ++i) {
    EXPECT_EQ(obstacle_map.data[i], expect_obstacle_map_data[i]);
  }

  // Test of TMC type distance_map
  const tmc_navigation_msgs::msg::OccupancyGridUint tmc_distance_map = test_node_->tmc_distance_map().get();
  // Test of meta data
  EXPECT_FLOAT_EQ(tmc_distance_map.info.resolution, map_resolution);
  EXPECT_EQ(tmc_distance_map.info.width, map_width);
  EXPECT_EQ(tmc_distance_map.info.height, map_height);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.position.x, map_origin_x);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.position.y, map_origin_y);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.position.z, kMapOriginZ);
  orientation = CreateQuaternionMsgFromYaw(map_origin_yaw);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.orientation.x, orientation.x);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.orientation.y, orientation.y);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.orientation.z, orientation.z);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.orientation.w, orientation.w);
  // Test of header
  EXPECT_STREQ(tmc_distance_map.header.frame_id.c_str(), kFrameId);
  // Test of data
  const std::vector<int64_t> expect_tmc_distance_map_data = test_node_->expect_tmc_distance_map_data();
  ASSERT_EQ(expect_tmc_distance_map_data.size(), tmc_distance_map.data.size());
  for (uint32_t i = 0; i < tmc_distance_map.info.width * tmc_distance_map.info.height; ++i) {
    EXPECT_EQ(tmc_distance_map.data[i], expect_tmc_distance_map_data[i]);
  }

  // Test of TMC type obstacle_map
  const tmc_navigation_msgs::msg::OccupancyGridUint tmc_obstacle_map = test_node_->tmc_obstacle_map().get();
  // Test of meta data
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.resolution, map_resolution);
  EXPECT_EQ(tmc_obstacle_map.info.width, map_width);
  EXPECT_EQ(tmc_obstacle_map.info.height, map_height);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.position.x, map_origin_x);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.position.y, map_origin_y);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.position.z, kMapOriginZ);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.orientation.x, orientation.x);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.orientation.y, orientation.y);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.orientation.z, orientation.z);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.orientation.w, orientation.w);
  // Test of header
  EXPECT_STREQ(tmc_obstacle_map.header.frame_id.c_str(), kFrameId);
  // Test of data
  const std::vector<int64_t> expect_tmc_obstacle_map_data = test_node_->expect_tmc_obstacle_map_data();
  ASSERT_EQ(expect_tmc_obstacle_map_data.size(), tmc_obstacle_map.data.size());
  for (uint32_t i = 0; i < tmc_obstacle_map.info.width * tmc_obstacle_map.info.height; ++i) {
    EXPECT_EQ(tmc_obstacle_map.data[i], expect_tmc_obstacle_map_data[i]);
  }
}

/// Test of map online update function
TEST_F(TestMapServer, OnlineUpdateMap) {
  // Spin until subscribing to the old map
  ASSERT_TRUE(WaitForMapUpdate());

  // Lower the subscribe flag once subscribed
  test_node_->ResetSubscribeMaps();
  /// Publish map topic for online updates
  test_node_->PublishMapTopic();
  // Spin until subscribing to the new map
  ASSERT_TRUE(WaitForMapUpdate());
  const double map_resolution = test_node_->map_resolution();
  const int32_t map_width = test_node_->map_width();
  const int32_t map_height = test_node_->map_height();
  const double map_origin_x = test_node_->map_origin_x();
  const double map_origin_y = test_node_->map_origin_y();
  const double map_origin_yaw = test_node_->map_origin_yaw();
  // Test of distance_map
  const nav_msgs::msg::OccupancyGrid distance_map = test_node_->distance_map().get();
  // Test of meta data
  EXPECT_FLOAT_EQ(distance_map.info.resolution, map_resolution);
  EXPECT_EQ(distance_map.info.width, map_width);
  EXPECT_EQ(distance_map.info.height, map_height);
  EXPECT_FLOAT_EQ(distance_map.info.origin.position.x, map_origin_x);
  EXPECT_FLOAT_EQ(distance_map.info.origin.position.y, map_origin_y);
  EXPECT_FLOAT_EQ(distance_map.info.origin.position.z, kMapOriginZ);
  geometry_msgs::msg::Quaternion orientation;
  orientation = CreateQuaternionMsgFromYaw(map_origin_yaw);
  EXPECT_FLOAT_EQ(distance_map.info.origin.orientation.x, orientation.x);
  EXPECT_FLOAT_EQ(distance_map.info.origin.orientation.y, orientation.y);
  EXPECT_FLOAT_EQ(distance_map.info.origin.orientation.z, orientation.z);
  EXPECT_FLOAT_EQ(distance_map.info.origin.orientation.w, orientation.w);
  // Test of header
  EXPECT_STREQ(distance_map.header.frame_id.c_str(), kFrameId);
  // Test of data
  const std::vector<int64_t> expect_distance_map_data = test_node_->update_map_data();
  ASSERT_EQ(expect_distance_map_data.size(), distance_map.data.size());
  for (uint32_t i = 0; i < distance_map.info.width * distance_map.info.height; ++i) {
    EXPECT_EQ(distance_map.data[i], expect_distance_map_data[i]);
  }

  // Test of obstacle_map
  const nav_msgs::msg::OccupancyGrid obstacle_map = test_node_->obstacle_map().get();
  // Test of meta data
  EXPECT_FLOAT_EQ(obstacle_map.info.resolution, map_resolution);
  EXPECT_EQ(obstacle_map.info.width, map_width);
  EXPECT_EQ(obstacle_map.info.height, map_height);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.position.x, map_origin_x);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.position.y, map_origin_y);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.position.z, kMapOriginZ);
  orientation = CreateQuaternionMsgFromYaw(map_origin_yaw);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.orientation.x, orientation.x);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.orientation.y, orientation.y);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.orientation.z, orientation.z);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.orientation.w, orientation.w);
  // Test of header
  EXPECT_STREQ(obstacle_map.header.frame_id.c_str(), kFrameId);
  // Test of data
  const std::vector<int64_t> expect_obstacle_map_data = test_node_->update_map_data();
  ASSERT_EQ(expect_obstacle_map_data.size(), obstacle_map.data.size());
  for (uint32_t i = 0; i < obstacle_map.info.width * obstacle_map.info.height; ++i) {
    EXPECT_EQ(obstacle_map.data[i], expect_obstacle_map_data[i]);
  }

  // Test of TMC type distance_map
  const tmc_navigation_msgs::msg::OccupancyGridUint tmc_distance_map = test_node_->tmc_distance_map().get();
  // Test of meta data
  EXPECT_FLOAT_EQ(tmc_distance_map.info.resolution, map_resolution);
  EXPECT_EQ(tmc_distance_map.info.width, map_width);
  EXPECT_EQ(tmc_distance_map.info.height, map_height);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.position.x, map_origin_x);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.position.y, map_origin_y);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.position.z, kMapOriginZ);
  orientation = CreateQuaternionMsgFromYaw(map_origin_yaw);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.orientation.x, orientation.x);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.orientation.y, orientation.y);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.orientation.z, orientation.z);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.orientation.w, orientation.w);
  // Test of header
  EXPECT_STREQ(tmc_distance_map.header.frame_id.c_str(), kFrameId);
  // Test of data
  const std::vector<int64_t> expect_tmc_distance_map_data = test_node_->expect_update_tmc_map_data();
  ASSERT_EQ(expect_tmc_distance_map_data.size(), tmc_distance_map.data.size());
  for (uint32_t i = 0; i < tmc_distance_map.info.width * tmc_distance_map.info.height; ++i) {
    EXPECT_EQ(tmc_distance_map.data[i], expect_tmc_distance_map_data[i]);
  }

  // Test of TMC type obstacle_map
  const tmc_navigation_msgs::msg::OccupancyGridUint tmc_obstacle_map = test_node_->tmc_obstacle_map().get();
  // Test of meta data
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.resolution, map_resolution);
  EXPECT_EQ(tmc_obstacle_map.info.width, map_width);
  EXPECT_EQ(tmc_obstacle_map.info.height, map_height);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.position.x, map_origin_x);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.position.y, map_origin_y);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.position.z, kMapOriginZ);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.orientation.x, orientation.x);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.orientation.y, orientation.y);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.orientation.z, orientation.z);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.orientation.w, orientation.w);
  // Test of header
  EXPECT_STREQ(tmc_obstacle_map.header.frame_id.c_str(), kFrameId);
  // Test of data
  const std::vector<int64_t> expect_tmc_obstacle_map_data = test_node_->expect_update_tmc_map_data();
  ASSERT_EQ(expect_tmc_obstacle_map_data.size(), tmc_obstacle_map.data.size());
  for (uint32_t i = 0; i < tmc_obstacle_map.info.width * tmc_obstacle_map.info.height; ++i) {
    EXPECT_EQ(tmc_obstacle_map.data[i], expect_tmc_obstacle_map_data[i]);
  }
}

/// Test of map online update function
/// Verify that map reload from service is functioning correctly
TEST_F(TestMapServer, ReloadMap) {
  // Spin until subscribing to the old map
  ASSERT_TRUE(WaitForMapUpdate());
  // Lower the subscribe flag once subscribed
  test_node_->ResetSubscribeMaps();
  // Call map reload service
  // Read the configuration file that specifies the distance and obstacle images in reverse
  test_node_->ReloadReverseMap();
  // Spin until subscribing to the new map
  ASSERT_TRUE(WaitForMapUpdate());
  const double map_resolution = test_node_->map_resolution();
  const int32_t map_width = test_node_->map_width();
  const int32_t map_height = test_node_->map_height();
  const double map_origin_x = test_node_->map_origin_x();
  const double map_origin_y = test_node_->map_origin_y();
  const double map_origin_yaw = test_node_->map_origin_yaw();
  // Test of distance_map
  const nav_msgs::msg::OccupancyGrid distance_map = test_node_->distance_map().get();
  // Test of meta data
  EXPECT_FLOAT_EQ(distance_map.info.resolution, map_resolution);
  EXPECT_EQ(distance_map.info.width, map_width);
  EXPECT_EQ(distance_map.info.height, map_height);
  EXPECT_FLOAT_EQ(distance_map.info.origin.position.x, map_origin_x);
  EXPECT_FLOAT_EQ(distance_map.info.origin.position.y, map_origin_y);
  EXPECT_FLOAT_EQ(distance_map.info.origin.position.z, kMapOriginZ);
  geometry_msgs::msg::Quaternion orientation;
  orientation = CreateQuaternionMsgFromYaw(map_origin_yaw);
  EXPECT_FLOAT_EQ(distance_map.info.origin.orientation.x, orientation.x);
  EXPECT_FLOAT_EQ(distance_map.info.origin.orientation.y, orientation.y);
  EXPECT_FLOAT_EQ(distance_map.info.origin.orientation.z, orientation.z);
  EXPECT_FLOAT_EQ(distance_map.info.origin.orientation.w, orientation.w);
  // Test of header
  EXPECT_STREQ(distance_map.header.frame_id.c_str(), kFrameId);
  // Test of data
  // Verify if it matches the data of expect_obstacle_map_data
  const std::vector<int64_t> expect_distance_map_data = test_node_->expect_obstacle_map_data();
  ASSERT_EQ(expect_distance_map_data.size(), distance_map.data.size());
  for (uint32_t i = 0; i < distance_map.info.width * distance_map.info.height; ++i) {
    EXPECT_EQ(distance_map.data[i], expect_distance_map_data[i]);
  }

  // Test of obstacle_map
  const nav_msgs::msg::OccupancyGrid obstacle_map = test_node_->obstacle_map().get();
  // Test of meta data
  EXPECT_FLOAT_EQ(obstacle_map.info.resolution, map_resolution);
  EXPECT_EQ(obstacle_map.info.width, map_width);
  EXPECT_EQ(obstacle_map.info.height, map_height);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.position.x, map_origin_x);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.position.y, map_origin_y);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.position.z, kMapOriginZ);
  orientation = CreateQuaternionMsgFromYaw(map_origin_yaw);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.orientation.x, orientation.x);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.orientation.y, orientation.y);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.orientation.z, orientation.z);
  EXPECT_FLOAT_EQ(obstacle_map.info.origin.orientation.w, orientation.w);
  // Test of header
  EXPECT_STREQ(obstacle_map.header.frame_id.c_str(), kFrameId);
  // Test of data
  // Verify if it matches the data of expect_distance_map_data
  const std::vector<int64_t> expect_obstacle_map_data = test_node_->expect_distance_map_data();
  ASSERT_EQ(expect_obstacle_map_data.size(), obstacle_map.data.size());
  for (uint32_t i = 0; i < obstacle_map.info.width * obstacle_map.info.height; ++i) {
    EXPECT_EQ(obstacle_map.data[i], expect_obstacle_map_data[i]);
  }

  // Test of TMC type distance_map
  const tmc_navigation_msgs::msg::OccupancyGridUint tmc_distance_map = test_node_->tmc_distance_map().get();
  // Test of meta data
  EXPECT_FLOAT_EQ(tmc_distance_map.info.resolution, map_resolution);
  EXPECT_EQ(tmc_distance_map.info.width, map_width);
  EXPECT_EQ(tmc_distance_map.info.height, map_height);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.position.x, map_origin_x);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.position.y, map_origin_y);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.position.z, kMapOriginZ);
  orientation = CreateQuaternionMsgFromYaw(map_origin_yaw);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.orientation.x, orientation.x);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.orientation.y, orientation.y);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.orientation.z, orientation.z);
  EXPECT_FLOAT_EQ(tmc_distance_map.info.origin.orientation.w, orientation.w);
  // Test of header
  EXPECT_STREQ(tmc_distance_map.header.frame_id.c_str(), kFrameId);
  // Test of data
  // Verify if it matches the data of expect_tmc_obstacle_map_data
  const std::vector<int64_t> expect_tmc_distance_map_data = test_node_->expect_tmc_obstacle_map_data();
  ASSERT_EQ(expect_tmc_distance_map_data.size(), tmc_distance_map.data.size());
  for (uint32_t i = 0; i < tmc_distance_map.info.width * tmc_distance_map.info.height; ++i) {
    EXPECT_EQ(tmc_distance_map.data[i], expect_tmc_distance_map_data[i]);
  }

  // Test of TMC type obstacle_map
  const tmc_navigation_msgs::msg::OccupancyGridUint tmc_obstacle_map = test_node_->tmc_obstacle_map().get();
  // Test of meta data
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.resolution, map_resolution);
  EXPECT_EQ(tmc_obstacle_map.info.width, map_width);
  EXPECT_EQ(tmc_obstacle_map.info.height, map_height);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.position.x, map_origin_x);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.position.y, map_origin_y);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.position.z, kMapOriginZ);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.orientation.x, orientation.x);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.orientation.y, orientation.y);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.orientation.z, orientation.z);
  EXPECT_FLOAT_EQ(tmc_obstacle_map.info.origin.orientation.w, orientation.w);
  // Test of header
  EXPECT_STREQ(tmc_obstacle_map.header.frame_id.c_str(), kFrameId);
  // Test of data
  // Verify if it matches the data of expect_tmc_distance_map_data
  const std::vector<int64_t> expect_tmc_obstacle_map_data = test_node_->expect_tmc_distance_map_data();
  ASSERT_EQ(expect_tmc_obstacle_map_data.size(), tmc_obstacle_map.data.size());
  for (uint32_t i = 0; i < tmc_obstacle_map.info.width * tmc_obstacle_map.info.height; ++i) {
    EXPECT_EQ(tmc_obstacle_map.data[i], expect_tmc_obstacle_map_data[i]);
  }
}
}  // namespace tmc_grid_map_server

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::string parameter_file = std::string(getenv("PARAMETER_FILE"));
  parameter_file.erase(0, 1);
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);
  const std::string yaml_directory = ament_index_cpp::get_package_share_directory("tmc_grid_map_server") +
      "/test/parameter/";
  // Generate grid_map_server node
  auto grid_map_server_node = std::make_shared<tmc_grid_map_server::GridMapServerNode>(option);
  LoadParameterFromYaml(grid_map_server_node, yaml_directory, parameter_file);
  // Convert map_yaml_path parameter to absolute path and overwrite
  rclcpp::Parameter map_yaml_path_param;
  grid_map_server_node->get_parameter("map_yaml_path", map_yaml_path_param);
  std::string map_yaml_path = ament_index_cpp::get_package_share_directory("tmc_grid_map_server") +
      "/test/maps/" + map_yaml_path_param.as_string();
  rclcpp::Parameter map_yaml_path_param_abs("map_yaml_path", map_yaml_path);
  grid_map_server_node->set_parameter(map_yaml_path_param_abs);
  grid_map_server_node->Init();
  grid_map_server_node->Run();
  auto grid_map_server_node_thread = std::make_shared<std::thread>([&]() {
      rclcpp::spin(grid_map_server_node);
      });
  grid_map_server_node_thread->detach();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
