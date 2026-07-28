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
/// @file     pose_integrator-test.cpp
/// @brief    pose_integrator test (*GoogleTest)
/// @version  0.2.0
/// @author   Takao Yasuda
/// @author   Applied for Partner-Robot Coding Rule(Ver:x.xx)
/// @date     2012.05.08

#include <cmath>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "../src/pose_integrator_node.hpp"

namespace {
/// Number of test executions
constexpr uint32_t kTestCount = 3;
/// Topic buffer size (Publish)
constexpr uint32_t kTopicBufferSizePublish = 1;
/// Topic buffer size (Subscribe)
constexpr uint32_t kTopicBufferSizeSubscribe = 1;
/// Node name
constexpr const char* const kNodeName = "pose_integrator_test";
/// Topic name (Global self-position)
constexpr const char* const kTopicNameGlobalPose = "global_pose";
/// Topic name (Laser self-position)
constexpr const char* const kTopicNameLaserPose = "laser_2d_pose";
/// Travel speed in the X direction [m/s]
constexpr double kLinearVelX = 0.3;
/// Value of the cycle_time parameter set in pose_integrator
constexpr double kCycleTimeParam = 0.01;
/// Value of the convergence_time parameter set in pose_integrator
constexpr double kConvergenceTimeParam = 5.0;

// TODO(syuuhei_shiro): tmc_rostest_utilをROS2化してそこに置く
// Load parameters from the yaml file
void LoadParameterFromYaml(std::shared_ptr<rclcpp::Node> node,
    const std::string& yaml_directory, const std::string& yaml_name) {
  const std::string yaml_path = yaml_directory + yaml_name;
  // Load yaml and generate ParameterMap
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_params_t* yaml_params = rcl_yaml_node_struct_init(allocator);
  rcl_parse_yaml_file(yaml_path.c_str(), yaml_params);
  rclcpp::ParameterMap yaml_param_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);
  // Set ROS parameters to the node
  const std::string parameter_space = "/" + std::string(node->get_name());
  auto iter = yaml_param_map.find(parameter_space);
  for (auto& param : iter->second) {
    node->declare_parameter(param.get_name(), param.get_type());
    node->set_parameter(param);
  }
}

geometry_msgs::msg::TransformStamped CreateTransformStamped(const tf2::Transform transform, const rclcpp::Time& stamp,
    const std::string& frame_id, const std::string& child_frame_id) {
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = stamp;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  tf2::convert(transform, transform_stamped.transform);
  return transform_stamped;
}

tf2::Quaternion CreateQuaternionFromYaw(const double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return q;
}
}  // anonymous namespace

namespace tmc_pose_integrator {
using std::placeholders::_1;

/// Test node
class TestNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node(kNodeName, options) {}
  /// Initialization
  void Init() {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
    pub_laser2d_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        kTopicNameLaserPose, kTopicBufferSizePublish);
    sub_global_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(kTopicNameGlobalPose,
        kTopicBufferSizeSubscribe, std::bind(&TestNode::CallbackGlobalPose_, this, _1));
  }

  void PublishLaser2dPose(const geometry_msgs::msg::PoseWithCovarianceStamped& laser2d_pose) {
    pub_laser2d_pose_->publish(laser2d_pose);
  }

  void SendTransform(const geometry_msgs::msg::TransformStamped& transform) {
    tf_broadcaster_->sendTransform(transform);
  }

  geometry_msgs::msg::PoseStamped global_pose() { return global_pose_; }

 private:
  /// Callback (self-position estimation)
  void CallbackGlobalPose_(const geometry_msgs::msg::PoseStamped::SharedPtr global_pose) {
    global_pose_ = *global_pose;
  }

  /// Topic transmission (self-position estimation using 2D laser data)
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_laser2d_pose_;
  /// Topic reception (self-position estimation)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_global_pose_;
  /// Self-position estimation
  geometry_msgs::msg::PoseStamped global_pose_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

/// Class (pose_integrator_test)
class PoseIntegratorTest : public ::testing::Test {
 public:
  /// Constructor
  PoseIntegratorTest() {}
  /// Destructor
  ~PoseIntegratorTest() {}

 protected:
  /// Setup
  virtual void SetUp() {
    rate_ = std::make_shared<rclcpp::Rate>(1.0 / kCycleTimeParam);
    rclcpp::NodeOptions option;
    option.allow_undeclared_parameters();
    option.automatically_declare_parameters_from_overrides(true);
    test_node_ = std::make_shared<TestNode>(option);
    test_node_->Init();
  }
  /// Teardown
  virtual void TearDown() {}

  // Spin the test node
  void SpinOnce() {
    rate_->sleep();
    rclcpp::spin_some(test_node_);
  }

  std::shared_ptr<TestNode> test_node_;
  std::shared_ptr<rclcpp::Rate> rate_;
};

/// x=rand(), y=rand(), θ=divided into 4 (0°, 90°, 180°, 270°: 0[rad] to 2PI[rad])
/// @todo If possible, introduce the MT (Mersenne Twister) method for random number generation
TEST_F(PoseIntegratorTest, Random) {
  tf2::Transform odom;
  odom.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  odom.setRotation(CreateQuaternionFromYaw(0.0));

  double odom_x = 0.0;

  // Random test
  for (uint32_t i = 0; i < kTestCount; ++i) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "*****");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "Loop = " << i);
    // Random number generation (x=y:1 to 100, theta:0, 90°, 180°, 270°)
    double random_rad = static_cast<double>((random() % 4) * 90) * M_PI / 180;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "random_rad = " << random_rad);
    Eigen::Vector3d position = Eigen::Vector3d::Identity();
    position << (random() % 10000 + 1) * 0.01, (random() % 10000 + 1) * 0.01, 0.0;
    Eigen::Vector4d quaternion = Eigen::Vector4d::Identity();
    quaternion << 0.0, 0.0, sin(random_rad * 0.5), cos(random_rad * 0.5);

    // Initialize laser self-position
    geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance;
    pose_with_covariance.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    pose_with_covariance.header.frame_id = "test";
    pose_with_covariance.pose.pose.position.x = position.x();
    pose_with_covariance.pose.pose.position.y = position.y();
    pose_with_covariance.pose.pose.position.z = position.z();
    pose_with_covariance.pose.pose.orientation.x = quaternion.x();
    pose_with_covariance.pose.pose.orientation.y = quaternion.y();
    pose_with_covariance.pose.pose.orientation.z = quaternion.z();
    pose_with_covariance.pose.pose.orientation.w = quaternion.w();

    // Add laser self-position
    test_node_->PublishLaser2dPose(pose_with_covariance);
    SpinOnce();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "pose_with_covariance = \n" <<
        "position " <<
        pose_with_covariance.pose.pose.position.x << ", " <<
        pose_with_covariance.pose.pose.position.y << ", " <<
        pose_with_covariance.pose.pose.position.z << "\n" <<
        "orientation " <<
        pose_with_covariance.pose.pose.orientation.x << ", " <<
        pose_with_covariance.pose.pose.orientation.y << ", " <<
        pose_with_covariance.pose.pose.orientation.z << ", " <<
        pose_with_covariance.pose.pose.orientation.w);

    // Due to the construction of pose_integrator, the first time is skipped for time adjustment, so retransmit
    pose_with_covariance.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    test_node_->PublishLaser2dPose(pose_with_covariance);
    // Initialize odometry
    test_node_->SendTransform(CreateTransformStamped(
        odom, rclcpp::Clock(RCL_ROS_TIME).now(), "odom", "base_footprint"));
    SpinOnce();

    uint32_t loop_count = 0;
    geometry_msgs::msg::PoseStamped global_pose_1;
    geometry_msgs::msg::PoseStamped global_pose_2;
    geometry_msgs::msg::PoseStamped global_pose_3;
    bool is_get_global_pose_1 = false;
    bool is_get_global_pose_2 = false;
    bool is_get_global_pose_3 = false;
    while (rclcpp::ok()) {
      // Input odometry
      // Move straight in the x-axis direction
      odom_x += kLinearVelX * kCycleTimeParam;
      odom.setOrigin(tf2::Vector3(odom_x, 0.0, 0.0));
      odom.setRotation(CreateQuaternionFromYaw(0.0));
      test_node_->SendTransform(CreateTransformStamped(
          odom, rclcpp::Clock(RCL_ROS_TIME).now(), "odom", "base_footprint"));
      SpinOnce();
      // Wait until convergence time (target - 1 cycle)
      if (!is_get_global_pose_1) {
        if (loop_count * kCycleTimeParam > kConvergenceTimeParam - kCycleTimeParam) {
          global_pose_1 = test_node_->global_pose();
          is_get_global_pose_1 = true;
        }
      }
      // Wait until convergence time (target)
      if (!is_get_global_pose_2) {
        if (loop_count * kCycleTimeParam > kConvergenceTimeParam) {
          global_pose_2 = test_node_->global_pose();
          is_get_global_pose_2 = true;
        }
      }
      // Wait until convergence time (target + 1 cycle)
      if (!is_get_global_pose_3) {
        if (loop_count * kCycleTimeParam > kConvergenceTimeParam + kCycleTimeParam) {
          global_pose_3 = test_node_->global_pose();
          is_get_global_pose_3 = true;
          break;
        }
      }
      ++loop_count;
    }

    // Confirm that self-position converges within ±1 cycle
    pose_with_covariance.pose.pose.position.x =
        pose_with_covariance.pose.pose.position.x + cos(random_rad) * kLinearVelX *
        (kConvergenceTimeParam + kCycleTimeParam);
    pose_with_covariance.pose.pose.position.y =
        pose_with_covariance.pose.pose.position.y + sin(random_rad) * kLinearVelX *
        (kConvergenceTimeParam + kCycleTimeParam);
    double diff_x1 = pose_with_covariance.pose.pose.position.x - global_pose_1.pose.position.x;
    double diff_y1 = pose_with_covariance.pose.pose.position.y - global_pose_1.pose.position.y;
    double diff_x2 = pose_with_covariance.pose.pose.position.x - global_pose_2.pose.position.x;
    double diff_y2 = pose_with_covariance.pose.pose.position.y - global_pose_2.pose.position.y;
    double diff_x3 = pose_with_covariance.pose.pose.position.x - global_pose_3.pose.position.x;
    double diff_y3 = pose_with_covariance.pose.pose.position.y - global_pose_3.pose.position.y;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "global_pose_1 = \n" <<
        "position " <<
        global_pose_1.pose.position.x << ", " <<
        global_pose_1.pose.position.y << ", " <<
        global_pose_1.pose.position.z << "\n" <<
        "orientation " <<
        global_pose_1.pose.orientation.x << ", " <<
        global_pose_1.pose.orientation.y << ", " <<
        global_pose_1.pose.orientation.z << ", " <<
        global_pose_1.pose.orientation.w);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "global_pose_2 = \n" <<
        "position " <<
        global_pose_2.pose.position.x << ", " <<
        global_pose_2.pose.position.y << ", " <<
        global_pose_2.pose.position.z << "\n" <<
        "orientation " <<
        global_pose_2.pose.orientation.x << ", " <<
        global_pose_2.pose.orientation.y << ", " <<
        global_pose_2.pose.orientation.z << ", " <<
        global_pose_2.pose.orientation.w);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "global_pose_3 = \n" <<
        "position " <<
        global_pose_3.pose.position.x << ", " <<
        global_pose_3.pose.position.y << ", " <<
        global_pose_3.pose.position.z << "\n" <<
        "orientation " <<
        global_pose_3.pose.orientation.x << ", " <<
        global_pose_3.pose.orientation.y << ", " <<
        global_pose_3.pose.orientation.z << ", " <<
        global_pose_3.pose.orientation.w);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "dx1 = " << diff_x1);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "dy1 = " << diff_y1);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "dx2 = " << diff_x2);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "dy2 = " << diff_y2);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "dx3 = " << diff_x3);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "dy3 = " << diff_y3);

    // Allowable range is 3 cycles of movement + 1mm
    // The basis for 3 cycles is that the test node and pose_integrator node
    // Allowable cycle shift due to lack of perfect synchronization
    double expectation = kCycleTimeParam * 3 * kLinearVelX * kConvergenceTimeParam + 0.001;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("pose_integrator-test"), "expectation = " << expectation);
    bool success = false;
    if ((std::abs(diff_x1) < expectation && std::abs(diff_y1) < expectation) ||
        (std::abs(diff_x2) < expectation && std::abs(diff_y2) < expectation) ||
        (std::abs(diff_x3) < expectation && std::abs(diff_y3) < expectation)) {
      success = true;
    }
    EXPECT_EQ(success, true);
  }
}
}  // end namespace tmc_pose_integrator


/// Main process
/// @param[in] argc Total number of arguments (including program name)
/// @param[in] argv Pointer array to argument strings
/// @retval EXIT_SUCCESS = 0 Success
/// @retval EXIT_FAILURE = 1 Failure
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);
  // Launch pose_integrator node
  auto pose_integrator_node = std::make_shared<tmc_pose_integrator::PoseIntegratorNode>(option);
  const std::string yaml_directory = ament_index_cpp::get_package_share_directory("tmc_pose_integrator") +
      "/test/parameter/";
  LoadParameterFromYaml(pose_integrator_node, yaml_directory, "pose_integrator-test.yaml");
  pose_integrator_node->Init();
  // Create a thread
  auto pose_integrator_node_thread = std::make_shared<std::thread>([&]() {
        while (rclcpp::ok()) {
          rclcpp::spin_some(pose_integrator_node);
          pose_integrator_node->UpdateGlobalPose();
        }
      });

  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  pose_integrator_node_thread->join();
  pose_integrator_node.reset();

  return result;
}
