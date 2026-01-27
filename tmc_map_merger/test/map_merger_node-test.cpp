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
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "../src/map.hpp"
#include "../src/map_merger_node.hpp"
#include "test_utils.hpp"

namespace {
constexpr const char* const kFixedFrameId = "map";
constexpr const char* const kOriginFrameId = "base";
constexpr const char* const kPointCloudFrameId = "head_rgbd_sensor_rgb_frame";
constexpr const char* const kLaserScanFrameId = "base_range_sensor_link";
constexpr double kTimeout = 5.0;
constexpr double kRate = 10.0;

struct StaticTF {
  StaticTF(const std::string& in_frame, const std::string& in_child_frame,
      const double in_x, const double in_y, const double in_z,
      const double in_qx, const double in_qy, const double in_qz, const double in_qw)
      : frame(in_frame), child_frame(in_child_frame), x(in_x), y(in_y), z(in_z),
        qx(in_qx), qy(in_qy), qz(in_qz), qw(in_qw) {}

  std::string frame;
  std::string child_frame;
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
};

const StaticTF kMapToBaseTF(kFixedFrameId, kOriginFrameId, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
const StaticTF kBaseToRgbdTF(kOriginFrameId, kPointCloudFrameId, 0.0, 0.0, 0.0, 0.5, -0.5, -0.5, 0.5);
const StaticTF kBaseToScanTF(kOriginFrameId, kLaserScanFrameId, 0.0, 0.0, 0.18, 0.0, 0.0, 0.0, 1.0);

}  // anonymous namespace

namespace tmc_map_merger {
using std::placeholders::_1;
/// Test node
class TestNode : public rclcpp::Node {
 public:
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options) {}

  void Init() {
    rate_ = std::make_shared<rclcpp::Rate>(kRate);
    buffer_ =
      std::make_shared<tf2_ros::Buffer>(this->get_clock());
    listener_ =
      std::make_shared<tf2_ros::TransformListener>(*buffer_);
    // Clear recept object
    msg_ptr_.reset();
    // Generate publishers
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("test_points", 1);
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("test_scan", 1);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  }

  void StartSubscription() {
    sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/merged_map", 1,
        std::bind(&TestNode::Callback, this, _1));
  }

#if (defined __GNUC__ && __GNUC__ >= 7)
  bool MapIsPublished() { return (msg_ptr_ != nullptr); }
#else
  bool MapIsPublished() { return msg_ptr_; }
#endif

  // Send StaticTF
  void SendStaticTransform(const StaticTF& static_tf) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = static_tf.frame;
    transform_stamped.child_frame_id = static_tf.child_frame;
    transform_stamped.transform.translation.x = static_tf.x;
    transform_stamped.transform.translation.y = static_tf.y;
    transform_stamped.transform.translation.z = static_tf.z;
    transform_stamped.transform.rotation.x = static_tf.qx;
    transform_stamped.transform.rotation.y = static_tf.qy;
    transform_stamped.transform.rotation.z = static_tf.qz;
    transform_stamped.transform.rotation.w = static_tf.qw;
    tf_static_broadcaster_->sendTransform(transform_stamped);
  }

  // Check if TF is convertible
  bool TFHasTree() {
    std::vector<std::pair<std::string, std::string> > tf_pairs;
    tf_pairs.push_back(std::make_pair(kFixedFrameId, kOriginFrameId));
    tf_pairs.push_back(std::make_pair(kOriginFrameId, kPointCloudFrameId));
    tf_pairs.push_back(std::make_pair(kOriginFrameId, kLaserScanFrameId));
    for (std::vector<std::pair<std::string, std::string> >::const_iterator it = tf_pairs.begin();
         it != tf_pairs.end(); ++it) {
      const bool can_transform = buffer_->canTransform(it->first, it->second, rclcpp::Time(0),
          rclcpp::Duration::from_seconds(1.0));
      if (!can_transform) return false;
    }
    return true;
  }

  // Check if connected to subscriber
  bool SensorsAreSubscribed() {
    return (cloud_pub_->get_subscription_count() != 0 && scan_pub_->get_subscription_count() != 0);
  }

  // Wait until some condition is met
  bool WaitUntil(std::function<bool()> condition_function, double timeout_sec) {
    // Error check for arguments
    if (!condition_function) {
      throw std::invalid_argument("Function for waiting is empty.");
    }
    if (timeout_sec < 0.0) {
      throw std::invalid_argument("Timeout must must have fully value");
    }

    const rclcpp::Time end_time = rclcpp::Clock(RCL_ROS_TIME).now() + rclcpp::Duration::from_seconds(timeout_sec);
    while (rclcpp::ok()) {
      SpinOnce();
      if (condition_function()) return true;
      if (rclcpp::Clock(RCL_ROS_TIME).now() >= end_time) break;
    }
    return false;
  }

  void SpinOnce() {
    rclcpp::spin_some(shared_from_this());
    rate_->sleep();
  }

  void PublishPointCloud(const sensor_msgs::msg::PointCloud2& cloud) {
    cloud_pub_->publish(cloud);
  }
  void PublishLaserScan(const sensor_msgs::msg::LaserScan& scan) {
    scan_pub_->publish(scan);
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr msg_ptr() { return msg_ptr_; }

 private:
  void Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_ptr) {
    msg_ptr_.reset(new nav_msgs::msg::OccupancyGrid(*msg_ptr));
  }

  std::shared_ptr<rclcpp::Rate> rate_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr msg_ptr_;

  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

class MapMergerNodeTest : public testing::Test {
 public:
  MapMergerNodeTest() {}
  virtual void SetUp() {
    test_node_ = std::make_shared<TestNode>();
    test_node_->Init();
    // Publish StaticTF
    test_node_->SendStaticTransform(kMapToBaseTF);
    test_node_->SendStaticTransform(kBaseToRgbdTF);
    test_node_->SendStaticTransform(kBaseToScanTF);
  }

  virtual void TearDown() {}

 protected:
  std::shared_ptr<TestNode> test_node_;
};

TEST_F(MapMergerNodeTest, NoSensorData) {
  // Setup
  // Nothing to do

  // Excersise
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return test_node_->TFHasTree(); }, kTimeout));
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return test_node_->SensorsAreSubscribed(); }, kTimeout));
  test_node_->StartSubscription();
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return test_node_->MapIsPublished(); }, kTimeout));

  // Verify
  nav_msgs::msg::OccupancyGrid::SharedPtr msg_ptr = test_node_->msg_ptr();
  ASSERT_NE(msg_ptr->data.size(), 0);
  for (size_t i = 0; i < msg_ptr->data.size(); ++i) {
    ASSERT_EQ(msg_ptr->data[i], kUnknown);
  }
}


TEST_F(MapMergerNodeTest, SendPointCloud) {
  // Setup
  pcl::PointCloud<pcl::PointXYZ> pcl_object;
  const int data_num = 100;
  for (int i = 0; i < data_num; ++i) {
    // 0 to 1.0 [m]
    const float translation = i / static_cast<float>(data_num);
    pcl_object.points.push_back(pcl::PointXYZ(translation, translation, translation));
  }
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pcl_object, msg);
  msg.header.frame_id = kPointCloudFrameId;
  msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  // Excersise
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return test_node_->TFHasTree(); }, kTimeout));
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return test_node_->SensorsAreSubscribed(); }, kTimeout));
  test_node_->PublishPointCloud(msg);
  test_node_->StartSubscription();
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return test_node_->MapIsPublished(); }, kTimeout));
  nav_msgs::msg::OccupancyGrid::SharedPtr msg_ptr = test_node_->msg_ptr();
  ASSERT_NE(msg_ptr->data.size(), 0);
  bool found_sensor_data = false;
  for (size_t i = 0; i < msg_ptr->data.size(); ++i) {
    if (msg_ptr->data[i] != kUnknown) {
      found_sensor_data = true;
      break;
    }
  }

  // Verify
  ASSERT_TRUE(found_sensor_data);
}


TEST_F(MapMergerNodeTest, SendScan) {
  // Setup
  sensor_msgs::msg::LaserScan msg;
  const int data_num = 100;
  msg.angle_max = 1.0;
  msg.angle_min = -msg.angle_max;
  msg.angle_increment = (msg.angle_max - msg.angle_min) / static_cast<float>(data_num);
  msg.range_max = 1.0;
  msg.range_min = 0.0;
  for (int i = 0; i < data_num; ++i) {
    // 0 to 1.0 [m]
    const float range = i / static_cast<float>(data_num);
    msg.ranges.push_back(range);
  }
  msg.header.frame_id = kLaserScanFrameId;
  msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  // Excersise
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return test_node_->TFHasTree(); }, kTimeout));
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return test_node_->SensorsAreSubscribed(); }, kTimeout));
  test_node_->PublishLaserScan(msg);
  test_node_->StartSubscription();
  ASSERT_TRUE(test_node_->WaitUntil([&]() { return test_node_->MapIsPublished(); }, kTimeout));
  nav_msgs::msg::OccupancyGrid::SharedPtr msg_ptr = test_node_->msg_ptr();
  ASSERT_NE(msg_ptr->data.size(), 0);
  bool found_sensor_data = false;
  for (size_t i = 0; i < msg_ptr->data.size(); ++i) {
    if (msg_ptr->data[i] != kUnknown) {
      found_sensor_data = true;
      break;
    }
  }

  // Verify
  ASSERT_TRUE(found_sensor_data);
}
}  // end of namespace tmc_map_merger

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Create map_merger node
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);
  auto map_merger_node = std::make_shared<tmc_map_merger::MapMergerNode>(option);
  // Set parameters
  const std::string yaml_directory = ament_index_cpp::get_package_share_directory("tmc_map_merger") +
      "/test/parameter/";
  LoadParameterFromYaml(map_merger_node, yaml_directory, "map_merger_node-test.yaml");
  map_merger_node->Init();
  auto map_merger_node_thread = std::make_shared<std::thread>([&]() {
      rclcpp::spin(map_merger_node);
      });
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  map_merger_node_thread->join();
  map_merger_node.reset();

  return result;
}
