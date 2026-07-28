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
#include "../src/base_velocity_optimizer.hpp"

#include <math.h>
#include <limits>
#include <angles/angles.h>
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "test_utils.hpp"

namespace {
// To confirm the reflection of settings for normal parameters, set values different from the default ones.
constexpr double kCollisionAreaRadius = 0.8;
constexpr double kCollisionAreaRadiusSmall = 0.2;
constexpr double kCollisionAreaIncreaseRate = 0.1;
constexpr double kEstimationTime = 2.0;
constexpr double kCollisionScoreThreshold = 3.0;
constexpr double kSearchDirectionRange = (45.0 / 180.0 * M_PI);
constexpr double kAvoidanceDirectionOffset = (5.0 / 180.0 * M_PI);
// Input velocity
constexpr double kInputVelocityX = 1.0;
constexpr double kInputVelocityY = 0.0;
constexpr double kInputVelocityT = (M_PI / 2.0);
// Obstacle placement angle step [rad]
constexpr double kObstacleDirectionStep = (7.5 / 180.0 * M_PI);
// Obstacle radius [m] Since the voxel filter is not used in this test, make it smaller for easier score setting.
constexpr double kObstacleRadius = 0.2;
constexpr double kObstacleRadiusSmall = 0.01;
// Number of obstacle points
constexpr uint32_t kObstaclePointNum = 10;
}  // anonymous namespace

namespace tmc_base_velocity_adjuster {
class TestNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options) {}

  /// Parameter settings
  void SetParameters(const double collision_area_radius,
                     const double collision_area_increase_rate,
                     const double estimation_time,
                     const double collision_score_threshold,
                     const double search_direction_range,
                     const double avoidance_direction_offset) {
    SetParameter("collision_estimator.collision_area_radius", collision_area_radius);
    SetParameter("collision_estimator.collision_area_increase_rate", collision_area_increase_rate);
    SetParameter("collision_estimator.estimation_time", estimation_time);
    SetParameter("base_velocity_optimizer.collision_score_threshold", collision_score_threshold);
    SetParameter("base_velocity_optimizer.search_direction_range", search_direction_range);
    SetParameter("base_velocity_optimizer.avoidance_direction_offset", avoidance_direction_offset);
  }

  void SetParameter(const std::string& name, const double value) {
    rclcpp::Parameter param(
        name, rclcpp::ParameterValue(value));
    this->declare_parameter(param.get_name(), param.get_type());
    this->set_parameter(param);
  }
};

/// BaseVelocityOptimizer test fixture
class BaseVelocityOptimizerTest : public ::testing::Test {
 public:
  BaseVelocityOptimizerTest() {}

 protected:
  virtual void SetUp() {
    rclcpp::NodeOptions option;
    option.allow_undeclared_parameters();
    option.automatically_declare_parameters_from_overrides(true);
    test_node_ = std::make_shared<TestNode>(option);
  }

  std::shared_ptr<TestNode> test_node_;
};

/// BaseVelocityOptimizer test
/// When the translational velocity of the input speed is 0 and only the rotational speed is present, issue the input speed as is.
TEST_F(BaseVelocityOptimizerTest, PassThroughWhenAngularVelocityOnly) {
  // setup
  // Parameter settings
  test_node_->SetParameters(kCollisionAreaRadius, kCollisionAreaIncreaseRate, kEstimationTime,
      kCollisionScoreThreshold, kSearchDirectionRange, kAvoidanceDirectionOffset);
  // Input speed with translational velocity 0
  const Eigen::Vector3d input_velocity(0.0, 0.0, kInputVelocityT);
  const Eigen::Vector3d estimation_point = input_velocity * kEstimationTime;
  // Obstacle settings
  PointCloud::Ptr obstacle(new PointCloud());
  AddCircularObstacle(estimation_point.x(), estimation_point.y(), kObstacleRadius,
                      kObstaclePointNum, *obstacle);

  // exercise
  // Generate the test target instance
  BaseVelocityOptimizer<pcl::PointCloud<pcl::PointXYZ>> optimizer(test_node_);
  Eigen::Vector3d output_velocity;
  optimizer.OptimizeVelocity(input_velocity, obstacle, output_velocity);

  // verify
  // Confirm the consistency between input speed and output speed
  EXPECT_DOUBLE_EQ(output_velocity.x(), input_velocity.x());
  EXPECT_DOUBLE_EQ(output_velocity.y(), input_velocity.y());
  EXPECT_DOUBLE_EQ(output_velocity.z(), input_velocity.z());
}

/// BaseVelocityOptimizer test
/// When the possibility of interference with obstacles is below the lower threshold, issue the input speed as is.
TEST_F(BaseVelocityOptimizerTest, PassThroughForFewObstacles) {
  // setup
  // Parameter settings
  test_node_->SetParameters(kCollisionAreaRadius, kCollisionAreaIncreaseRate, kEstimationTime,
      kCollisionScoreThreshold, kSearchDirectionRange, kAvoidanceDirectionOffset);
  // Input speed
  const Eigen::Vector3d input_velocity(kInputVelocityX, kInputVelocityY, kInputVelocityT);
  const Eigen::Vector3d estimation_point = input_velocity * kEstimationTime;
  // Obstacle settings Place one less point than the threshold.
  PointCloud::Ptr obstacle(new PointCloud());
  AddCircularObstacle(estimation_point.x(), estimation_point.y(), kObstacleRadius,
                      static_cast<uint32_t>(kCollisionScoreThreshold - 1.0), *obstacle);

  // exercise
  // Generate the test target instance
  BaseVelocityOptimizer<pcl::PointCloud<pcl::PointXYZ>> optimizer(test_node_);
  Eigen::Vector3d output_velocity;
  optimizer.OptimizeVelocity(input_velocity, obstacle, output_velocity);

  // verify
  // Confirm the consistency between input speed and output speed
  EXPECT_DOUBLE_EQ(output_velocity.x(), input_velocity.x());
  EXPECT_DOUBLE_EQ(output_velocity.y(), input_velocity.y());
  EXPECT_DOUBLE_EQ(output_velocity.z(), input_velocity.z());
}

/// BaseVelocityOptimizer test
/// When there is no bias in the score to the left or right, initially avoid in the direction specified by avoidance_direction_offset.
TEST_F(BaseVelocityOptimizerTest, ProioritizeAvoidanceDirectionOffset) {
  // setup
  // Parameter settings
  test_node_->SetParameters(kCollisionAreaRadius, kCollisionAreaIncreaseRate, kEstimationTime,
      kCollisionScoreThreshold, kSearchDirectionRange, kAvoidanceDirectionOffset);
  // Input speed
  const Eigen::Vector3d input_velocity(kInputVelocityX, kInputVelocityY, kInputVelocityT);
  const Eigen::Vector3d estimation_point = input_velocity * kEstimationTime;
  // Obstacle settings
  PointCloud::Ptr obstacle(new PointCloud());
  AddCircularObstacle(estimation_point.x(), estimation_point.y(), kObstacleRadius,
                      kObstaclePointNum, *obstacle);

  // exercise
  // Generate the test target instance
  BaseVelocityOptimizer<pcl::PointCloud<pcl::PointXYZ>> optimizer(test_node_);
  Eigen::Vector3d output_velocity;
  optimizer.OptimizeVelocity(input_velocity, obstacle, output_velocity);

  // verify
  // Confirm that the output speed is tilted in the direction (left) specified by avoidance_direction_offset.
  const double input_direction = atan2(input_velocity.y(), input_velocity.x());
  const double output_direction = atan2(output_velocity.y(), output_velocity.x());
  const double direction_diff = angles::shortest_angular_distance(input_direction, output_direction);
  EXPECT_GT(direction_diff * kAvoidanceDirectionOffset, 0.0);
}

/// BaseVelocityOptimizer test
/// When there is bias in the score to the left or right, avoid in the direction of the lower score (right).
TEST_F(BaseVelocityOptimizerTest, SelectDirectionToAvoidObstacleRight) {
  // setup
  // Parameter settings
  test_node_->SetParameters(kCollisionAreaRadius, kCollisionAreaIncreaseRate, kEstimationTime,
      kCollisionScoreThreshold, kSearchDirectionRange, kAvoidanceDirectionOffset);
  // Input speed
  const Eigen::Vector3d input_velocity(kInputVelocityX, kInputVelocityY, kInputVelocityT);
  // Obstacle settings Place obstacles skewed to the left of the direction of travel.
  const double input_direction = atan2(input_velocity.y(), input_velocity.x());
  const double obstacle_direction = input_direction + kObstacleDirectionStep;
  const Eigen::Vector3d obstacle_point(cos(obstacle_direction) * kEstimationTime,
                                       sin(obstacle_direction) * kEstimationTime, 0.0);
  PointCloud::Ptr obstacle(new PointCloud());
  AddCircularObstacle(obstacle_point.x(), obstacle_point.y(), kObstacleRadius,
                      kObstaclePointNum, *obstacle);

  // exercise
  // Generate the test target instance
  BaseVelocityOptimizer<pcl::PointCloud<pcl::PointXYZ>> optimizer(test_node_);
  Eigen::Vector3d output_velocity;
  optimizer.OptimizeVelocity(input_velocity, obstacle, output_velocity);

  // verify
  // Confirm that the output speed is tilted to the right.
  const double output_direction = atan2(output_velocity.y(), output_velocity.x());
  const double direction_diff = angles::shortest_angular_distance(input_direction, output_direction);
  EXPECT_LT(direction_diff, 0.0);
}

/// BaseVelocityOptimizer test
/// When there is bias in the score to the left or right, avoid in the direction of the lower score (left).
TEST_F(BaseVelocityOptimizerTest, SelectDirectionToAvoidObstacleLeft) {
  // setup
  // Parameter settings
  test_node_->SetParameters(kCollisionAreaRadius, kCollisionAreaIncreaseRate, kEstimationTime,
      kCollisionScoreThreshold, kSearchDirectionRange, kAvoidanceDirectionOffset);
  // Input speed
  const Eigen::Vector3d input_velocity(kInputVelocityX, kInputVelocityY, kInputVelocityT);
  // Obstacle settings Place obstacles skewed to the right of the direction of travel.
  const double input_direction = atan2(input_velocity.y(), input_velocity.x());
  const double obstacle_direction = input_direction - kObstacleDirectionStep;
  const Eigen::Vector3d obstacle_point(cos(obstacle_direction) * kEstimationTime,
                                       sin(obstacle_direction) * kEstimationTime, 0.0);
  PointCloud::Ptr obstacle(new PointCloud());
  AddCircularObstacle(obstacle_point.x(), obstacle_point.y(), kObstacleRadius,
                      kObstaclePointNum, *obstacle);

  // exercise
  // Generate the test target instance
  BaseVelocityOptimizer<pcl::PointCloud<pcl::PointXYZ>> optimizer(test_node_);
  Eigen::Vector3d output_velocity;
  optimizer.OptimizeVelocity(input_velocity, obstacle, output_velocity);

  // verify
  // Confirm that the output speed is tilted to the left.
  const double output_direction = atan2(output_velocity.y(), output_velocity.x());
  const double direction_diff = angles::shortest_angular_distance(input_direction, output_direction);
  EXPECT_GT(direction_diff, 0.0);
}

/// BaseVelocityOptimizer test
/// When there is no bias in the score to the left or right, avoid in the same direction as the previous time.
TEST_F(BaseVelocityOptimizerTest, PrioritizePreviousDirection) {
  // setup
  // Parameter settings
  test_node_->SetParameters(kCollisionAreaRadius, kCollisionAreaIncreaseRate, kEstimationTime,
      kCollisionScoreThreshold, kSearchDirectionRange, kAvoidanceDirectionOffset);
  // Input speed
  const Eigen::Vector3d input_velocity(kInputVelocityX, kInputVelocityY, kInputVelocityT);
  const Eigen::Vector3d estimation_point = input_velocity * kEstimationTime;
  // Obstacle settings Create two sets: skewed to the left of the direction of travel and directly in front.
  const double input_direction = atan2(input_velocity.y(), input_velocity.x());
  const double obstacle_direction = input_direction + kObstacleDirectionStep;
  const Eigen::Vector3d obstacle_point(cos(obstacle_direction) * kEstimationTime,
                                       sin(obstacle_direction) * kEstimationTime, 0.0);
  PointCloud::Ptr obstacle_left(new PointCloud());
  AddCircularObstacle(obstacle_point.x(), obstacle_point.y(), kObstacleRadius,
                      kObstaclePointNum, *obstacle_left);
  PointCloud::Ptr obstacle_front(new PointCloud());
  AddCircularObstacle(estimation_point.x(), estimation_point.y(), kObstacleRadius,
                      kObstaclePointNum, *obstacle_front);

  // exercise
  // First, input the obstacles skewed to the left to make it avoid to the right.
  // Generate the test target instance
  BaseVelocityOptimizer<pcl::PointCloud<pcl::PointXYZ>> optimizer(test_node_);
  Eigen::Vector3d first_output_velocity;
  optimizer.OptimizeVelocity(input_velocity, obstacle_left, first_output_velocity);

  // verify
  // Confirm that the output speed is tilted to the right.
  const double first_output_direction = atan2(first_output_velocity.y(), first_output_velocity.x());
  const double first_direction_diff = angles::shortest_angular_distance(input_direction, first_output_direction);
  EXPECT_LT(first_direction_diff, 0.0);

  // exercise
  // Next, input the obstacles directly in front.
  Eigen::Vector3d second_output_velocity;
  optimizer.OptimizeVelocity(input_velocity, obstacle_front, second_output_velocity);

  // verify
  // Confirm that the output speed is tilted to the right.
  const double second_output_direction = atan2(second_output_velocity.y(), second_output_velocity.x());
  const double second_direction_diff = angles::shortest_angular_distance(input_direction, second_output_direction);
  EXPECT_LT(second_direction_diff, 0.0);
}

/// BaseVelocityOptimizer test
/// The direction with the smallest obstacle score is selected.
/// If there are multiple directions with the same score, the direction closest to the input speed direction is prioritized.
TEST_F(BaseVelocityOptimizerTest, PriotizeInputDirectionAgainstTiedScores) {
  // For the direction directly in front, completely block the left side and place obstacles in the center of the search range on the right side to create two gaps.
  // Confirm that it tries to pass through the gap closer to the front.
  // setup
  // Parameter settings Since the direction settings for obstacles are detailed in this test, make both the interference area and obstacles smaller.
  test_node_->SetParameters(kCollisionAreaRadiusSmall, kCollisionAreaIncreaseRate, kEstimationTime,
      kCollisionScoreThreshold, kSearchDirectionRange, kAvoidanceDirectionOffset);
  // Input speed
  const Eigen::Vector3d input_velocity(kInputVelocityX, kInputVelocityY, kInputVelocityT);
  // Obstacle settings
  const double input_direction = atan2(input_velocity.y(), input_velocity.x());
  PointCloud::Ptr obstacle(new PointCloud());
  // Fill the range from directly in front to 90° to the left.
  for (int32_t i = 0; i < static_cast<int32_t>(M_PI / 2.0 / kObstacleDirectionStep); ++i) {
    const double obstacle_direction = input_direction + kObstacleDirectionStep * i;
    const Eigen::Vector3d front_to_left_obstacle_point(cos(obstacle_direction) * kEstimationTime,
                                                       sin(obstacle_direction) * kEstimationTime, 0.0);
    AddCircularObstacle(front_to_left_obstacle_point.x(), front_to_left_obstacle_point.y(), kObstacleRadiusSmall,
                        kObstaclePointNum, *obstacle);
  }
  // Block the center of the search range on the right side.
  const double obstacle_direction = input_direction - kSearchDirectionRange / 2.0;
  const Eigen::Vector3d right_obstacle_point(cos(obstacle_direction) * kEstimationTime,
                                       sin(obstacle_direction) * kEstimationTime, 0.0);
  AddCircularObstacle(right_obstacle_point.x(), right_obstacle_point.y(), kObstacleRadiusSmall,
                      kObstaclePointNum, *obstacle);

  // exercise
  // Generate the test target instance
  BaseVelocityOptimizer<pcl::PointCloud<pcl::PointXYZ>> optimizer(test_node_);
  Eigen::Vector3d output_velocity;
  optimizer.OptimizeVelocity(input_velocity, obstacle, output_velocity);

  // verify
  // Confirm that the output speed is between 0° and search range/2.
  const double output_direction = atan2(output_velocity.y(), output_velocity.x());
  const double direction_diff = angles::shortest_angular_distance(input_direction, output_direction);
  EXPECT_LT(direction_diff, 0.0);
  EXPECT_GT(direction_diff, -kSearchDirectionRange / 2.0);
}

/// BaseVelocityOptimizer test
/// If passage is not possible, the speed becomes stop speed.
TEST_F(BaseVelocityOptimizerTest, StopVelocityWhenUnalbleToPass) {
  // setup
  // Parameter settings
  test_node_->SetParameters(kCollisionAreaRadius, kCollisionAreaIncreaseRate, kEstimationTime,
      kCollisionScoreThreshold, kSearchDirectionRange, kAvoidanceDirectionOffset);
  // Input speed
  const Eigen::Vector3d input_velocity(kInputVelocityX, kInputVelocityY, kInputVelocityT);
  // Obstacle settings Fill the range of 90° to the left and right.
  const double input_direction = atan2(input_velocity.y(), input_velocity.x());
  PointCloud::Ptr obstacle(new PointCloud());
  for (int32_t i = -static_cast<int32_t>(M_PI / 2.0 / kObstacleDirectionStep);
       i < static_cast<int32_t>(M_PI / 2.0 / kObstacleDirectionStep); ++i) {
    const double obstacle_direction = input_direction + kObstacleDirectionStep * i;
    const Eigen::Vector3d obstacle_point(cos(obstacle_direction) * kEstimationTime,
                                         sin(obstacle_direction) * kEstimationTime, 0.0);
    AddCircularObstacle(obstacle_point.x(), obstacle_point.y(), kObstacleRadius,
                        kObstaclePointNum, *obstacle);
  }

  // exercise
  // Generate the test target instance
  BaseVelocityOptimizer<pcl::PointCloud<pcl::PointXYZ>> optimizer(test_node_);
  Eigen::Vector3d output_velocity;
  optimizer.OptimizeVelocity(input_velocity, obstacle, output_velocity);

  // verify
  // Confirm that the output speed becomes 0.
  EXPECT_DOUBLE_EQ(0.0,  output_velocity.norm());
}

/// BaseVelocityOptimizer test
/// If parameters are not set, default values are used.
TEST_F(BaseVelocityOptimizerTest, NoParametersSet) {
  // Since parameters are private members and cannot be read from the test, only confirm basic behavior.
  // setup
  // Do not set parameters.
  // Input speed
  const Eigen::Vector3d input_velocity(kInputVelocityX, kInputVelocityY, kInputVelocityT);
  // Obstacle settings
  // Distance that becomes the boundary of the interference judgment area according to the input speed.
  double obstacle_area_boundary_x =
      kInputVelocityX * kEstimationTimeDefault +
      kCollisionAreaRadiusDefault +
      kCollisionAreaIncreaseRateDefault * input_velocity.head(2).norm();
  // Obstacles on the boundary line of the interference judgment area.
  const Eigen::Vector3d obstacle_point_area_in(obstacle_area_boundary_x, 0.0, 0.0);
  PointCloud::Ptr obstacle_area_in(new PointCloud());
  AddCircularObstacle(obstacle_point_area_in.x(), obstacle_point_area_in.y(), kObstacleRadius,
                      kObstaclePointNum, *obstacle_area_in);
  // Obstacles outside the interference judgment area.
  const Eigen::Vector3d obstacle_point_area_out(obstacle_area_boundary_x + kObstacleRadius + kEpsilon, 0.0, 0.0);
  PointCloud::Ptr obstacle_area_out(new PointCloud());
  AddCircularObstacle(obstacle_point_area_out.x(), obstacle_point_area_out.y(), kObstacleRadius,
                      kObstaclePointNum, *obstacle_area_out);

  // exercise
  // Generate the test target instance
  BaseVelocityOptimizer<pcl::PointCloud<pcl::PointXYZ>> velocity_area_in_optimizer(test_node_);
  Eigen::Vector3d output_velocity_area_in;
  velocity_area_in_optimizer.OptimizeVelocity(input_velocity, obstacle_area_in, output_velocity_area_in);
  BaseVelocityOptimizer<pcl::PointCloud<pcl::PointXYZ>> velocity_area_out_optimizer(test_node_);
  Eigen::Vector3d output_velocity_area_out;
  velocity_area_out_optimizer.OptimizeVelocity(input_velocity, obstacle_area_out, output_velocity_area_out);

  // verify
  // Confirm that the speed is corrected for obstacles within the area.
  EXPECT_TRUE((std::abs(output_velocity_area_in.x() - input_velocity.x()) > std::numeric_limits<double>::epsilon()) ||
              (std::abs(output_velocity_area_in.y() - input_velocity.y()) > std::numeric_limits<double>::epsilon()));
  // Confirm that the speed is not corrected for obstacles outside the area.
  EXPECT_DOUBLE_EQ(input_velocity.x(), output_velocity_area_out.x());
  EXPECT_DOUBLE_EQ(input_velocity.y(), output_velocity_area_out.y());
}

/// BaseVelocityOptimizer test
/// If parameter settings are invalid, default values are used.
TEST_F(BaseVelocityOptimizerTest, InvalidParametersSet) {
  // Since parameters are private members and cannot be read from the test, only confirm basic behavior.
  // setup
  // Parameter settings
  test_node_->SetParameters(-1.0, -1.0, -1.0, -1.0, -1.0, -1.0);
  // Input speed
  const Eigen::Vector3d input_velocity(kInputVelocityX, kInputVelocityY, kInputVelocityT);
  // Obstacle settings
  // Distance that becomes the boundary of the interference judgment area according to the input speed.
  double obstacle_area_boundary_x =
      kInputVelocityX * kEstimationTimeDefault +
      kCollisionAreaRadiusDefault +
      kCollisionAreaIncreaseRateDefault * input_velocity.head(2).norm();
  // Obstacles on the boundary line of the interference judgment area.
  const Eigen::Vector3d obstacle_point_area_in(obstacle_area_boundary_x, 0.0, 0.0);
  PointCloud::Ptr obstacle_area_in(new PointCloud());
  AddCircularObstacle(obstacle_point_area_in.x(), obstacle_point_area_in.y(), kObstacleRadius,
                      kObstaclePointNum, *obstacle_area_in);
  // Obstacles outside the interference judgment area.
  const Eigen::Vector3d obstacle_point_area_out(obstacle_area_boundary_x + kObstacleRadius + kEpsilon, 0.0, 0.0);
  PointCloud::Ptr obstacle_area_out(new PointCloud());
  AddCircularObstacle(obstacle_point_area_out.x(), obstacle_point_area_out.y(), kObstacleRadius,
                      kObstaclePointNum, *obstacle_area_out);

  // exercise
  // Generate the test target instance
  BaseVelocityOptimizer<pcl::PointCloud<pcl::PointXYZ>> velocity_area_in_optimizer(test_node_);
  Eigen::Vector3d output_velocity_area_in;
  velocity_area_in_optimizer.OptimizeVelocity(input_velocity, obstacle_area_in, output_velocity_area_in);
  // Generate the test target instance
  BaseVelocityOptimizer<pcl::PointCloud<pcl::PointXYZ>> velocity_area_out_optimizer(test_node_);
  Eigen::Vector3d output_velocity_area_out;
  velocity_area_out_optimizer.OptimizeVelocity(input_velocity, obstacle_area_out, output_velocity_area_out);

  // verify
  // Confirm that the speed is corrected for obstacles within the area.
  EXPECT_TRUE((std::abs(output_velocity_area_in.x() - input_velocity.x()) > std::numeric_limits<double>::epsilon()) ||
              (std::abs(output_velocity_area_in.y() - input_velocity.y()) > std::numeric_limits<double>::epsilon()));
  // Confirm that the speed is not corrected for obstacles outside the area.
  EXPECT_DOUBLE_EQ(input_velocity.x(), output_velocity_area_out.x());
  EXPECT_DOUBLE_EQ(input_velocity.y(), output_velocity_area_out.y());
}

}  // namespace tmc_base_velocity_adjuster

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
