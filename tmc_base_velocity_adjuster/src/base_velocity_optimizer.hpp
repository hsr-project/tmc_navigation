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
/// @file base_velocity_optimizer.hpp
/// @brief Speed optimization adapted to obstacles
#ifndef TMC_BASE_VELOCITY_ADJUSTER_BASE_VELOCITY_OPTIMIZER_HPP_
#define TMC_BASE_VELOCITY_ADJUSTER_BASE_VELOCITY_OPTIMIZER_HPP_
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <angles/angles.h>
#include <rclcpp/rclcpp.hpp>

#include "collision_estimator.hpp"
#include "param.hpp"

namespace { // NOLINT
/// Search interval for velocity direction [rad]
constexpr double kSearchInterval = 0.01;
// Parameter name for interference judgment area radius
constexpr const char* const kCollisionAreaRadiusName = "collision_area_radius";
// Default value for interference judgment area radius [m]
constexpr double kCollisionAreaRadiusDefault = 0.5;
// Parameter name for interference judgment area radius increment rate
constexpr const char* const kCollisionAreaIncreaseRateName = "collision_area_increase_rate";
// Default value for interference judgment area radius increment rate [m/(m/s)]
constexpr double kCollisionAreaIncreaseRateDefault = 0.5;
// Parameter name for interference prediction time interval
constexpr const char* const kEstimationTimeName = "estimation_time";
// Default value for interference prediction time interval [s]
constexpr double kEstimationTimeDefault = 1.5;
// Parameter name for obstacle interference score threshold
constexpr const char* const kCollisionScoreThresholdName = "collision_score_threshold";
// Default value for obstacle interference score threshold
constexpr double kCollisionScoreThresholdDefault = 0.0;
// Parameter name for velocity direction search range
constexpr const char* const kSearchDirectionRangeName = "search_direction_range";
// Default value for velocity direction search range [rad]
constexpr double kSearchDirectionRangeDefault = 0.0;
// Parameter name for preferred avoidance direction offset
constexpr const char* const kAvoidanceDirectionOffsetName = "avoidance_direction_offset";
// Preferred avoidance direction offset [rad]
constexpr double kAvoidanceDirectionOffsetDefault = 0.0;
// Offset value for velocity direction prioritization based on previous correction direction [rad]
constexpr double kLastOffsetDirection = 0.3;
}  // anonymous namespace


namespace tmc_base_velocity_adjuster {


/// @brief Cart speed optimization class
/// @tparam Data Type of obstacle data used internally for optimization
template<class Data>
class BaseVelocityOptimizer {
 public:
  explicit BaseVelocityOptimizer(const rclcpp::Node::SharedPtr node) : last_adjusted_direction_(0.0) {
    // Parameter input
    GetParameters(node);
    // Create interference prediction class
    CreateEstimator(node);
  }

  ~BaseVelocityOptimizer() {}

  /// @brief Cart speed optimization
  /// @param [in] input_velocity Input cart speed
  /// @param [in] obstacle Obstacle information
  /// @param [out] optimized_velocity Optimized cart speed
  void OptimizeVelocity(const Eigen::Vector3d& input_velocity,
                        const typename Data::Ptr& obstacle,
                        Eigen::Vector3d& optimized_velocity) {
    const double input_direction = atan2(input_velocity(kY), input_velocity(kX));
    // If the magnitude of the translational component of the input speed is 0, or if the input speed is determined to have no interference, output the same speed
    if ((input_velocity.head(2).norm() < std::numeric_limits<double>::epsilon()) ||
        (estimator_->EstimateCollisionScore(input_velocity, obstacle) <= collision_score_threshold_)) {
      optimized_velocity = input_velocity;
      last_adjusted_direction_ = input_direction;
      return;
    }

    // If the difference between the previous correction direction and the input speed direction exceeds a certain threshold, add an offset to prioritize the previous correction direction
    const double diff_direction = angles::shortest_angular_distance(input_direction, last_adjusted_direction_);
    double offset = 0.0;
    if (std::abs(diff_direction) > kLastOffsetDirection) {
      const double sign = diff_direction / std::abs(diff_direction);
      offset = sign * kLastOffsetDirection;
    } else {
      // If the input direction is close to the previous correction direction, add an offset to prioritize the specified direction
      // Positive value: prioritize left direction, negative value: prioritize right direction
      offset = avoidance_direction_offset_;
    }
    // Search for velocity direction around the input speed direction and calculate the direction with the lowest interference score
    const int32_t di = static_cast<int32_t>(search_direction_range_ / kSearchInterval);
    double min_score = std::numeric_limits<double>::max();
    double optimized_direction = 0.0;
    for (int32_t i = -di; i <= di; ++i) {
      const auto ref_direction = static_cast<double>(i) * kSearchInterval;
      auto ref_velocity = input_velocity;
      ref_velocity.head(2) = Eigen::Rotation2Dd(ref_direction) * input_velocity.head(2);
      const auto collision_score = estimator_->EstimateCollisionScore(ref_velocity, obstacle);
      // If the scores are the same, adopt the direction closer to the input direction + offset
      if ((std::abs(collision_score - min_score) < std::numeric_limits<double>::epsilon()) &&
          (std::abs(angles::normalize_angle(ref_direction - offset)) <
           std::abs(angles::normalize_angle(optimized_direction - offset)))) {
        optimized_direction = ref_direction;
      } else if (collision_score <  min_score) {
        optimized_direction = ref_direction;
        min_score = collision_score;
      }
    }
    // If the minimum score is greater than the threshold, i.e., interference occurs, output stop speed as no solution
    if (min_score > collision_score_threshold_) {
      optimized_velocity = Eigen::Vector3d::Zero();
      last_adjusted_direction_ = input_direction;
      return;
    }
    // Output the speed in the direction with the minimum score
    optimized_velocity = input_velocity;
    optimized_velocity.head(2) = Eigen::Rotation2Dd(optimized_direction) * input_velocity.head(2);
    last_adjusted_direction_ = atan2(optimized_velocity(kY), optimized_velocity(kX));
    return;
  }

 private:
  /// @brief Create Estimator by inputting parameters
  /// @param [in] node Node
  void CreateEstimator(const rclcpp::Node::SharedPtr node) {
    // Parameter acquisition
    std::map<std::string, rclcpp::Parameter> collision_estimator_params;
    GetGroupParam(node, "collision_estimator", collision_estimator_params);

    double collision_area_radius = 0.0;
    GetOptionalParam(collision_estimator_params, kCollisionAreaRadiusName,
        collision_area_radius, kCollisionAreaRadiusDefault);
    if (collision_area_radius <= 0.0) {
      RCLCPP_WARN(node->get_logger(), "Parameter [%s] is not set or invalid. Use default value",
          kCollisionAreaRadiusName);
      collision_area_radius = kCollisionAreaRadiusDefault;
    }
    double collision_area_increase_rate = 0.0;
    GetOptionalParam(collision_estimator_params, kCollisionAreaIncreaseRateName,
        collision_area_increase_rate, kCollisionAreaIncreaseRateDefault);
    if (collision_area_increase_rate <= 0.0) {
      RCLCPP_WARN(node->get_logger(), "Parameter [%s] is not set or invalid. Use default value",
          kCollisionAreaIncreaseRateName);
      collision_area_increase_rate = kCollisionAreaIncreaseRateDefault;
    }

    double estimation_time = 0.0;
    GetOptionalParam(collision_estimator_params, kEstimationTimeName,
        estimation_time, kEstimationTimeDefault);
    if (estimation_time <= 0.0) {
      RCLCPP_WARN(node->get_logger(), "Parameter [%s] is not set or invalid. Use default value",
          kEstimationTimeName);
      estimation_time = kEstimationTimeDefault;
    }
    estimator_.reset(new CollisionEstimator<Data>(collision_area_radius,
                                                  collision_area_increase_rate,
                                                  estimation_time));
  }

  /// @brief Input parameters into members
  /// @param [in] node Node
  void GetParameters(const rclcpp::Node::SharedPtr node) {
    std::map<std::string, rclcpp::Parameter> optimizer_params;
    GetGroupParam(node, "base_velocity_optimizer", optimizer_params);
    GetOptionalParam(optimizer_params, kCollisionScoreThresholdName,
        collision_score_threshold_, kCollisionScoreThresholdDefault);
    if (collision_score_threshold_ < 0.0) {
      RCLCPP_WARN(node->get_logger(), "Parameter [%s] is not set or invalid. Use default value",
          kCollisionScoreThresholdName);
      collision_score_threshold_ = kCollisionScoreThresholdDefault;
    }
    GetOptionalParam(optimizer_params, kSearchDirectionRangeName,
        search_direction_range_, kSearchDirectionRangeDefault);
    if (search_direction_range_ < 0.0) {
      RCLCPP_WARN(node->get_logger(), "Parameter [%s] is not set or invalid. Use default value",
          kSearchDirectionRangeName);
      search_direction_range_ = kSearchDirectionRangeDefault;
    }
    GetOptionalParam(optimizer_params, kAvoidanceDirectionOffsetName,
        avoidance_direction_offset_, kAvoidanceDirectionOffsetDefault);
    if (avoidance_direction_offset_ < 0.0) {
      RCLCPP_WARN(node->get_logger(), "Parameter [%s] is not set or invalid. Use default value",
          kAvoidanceDirectionOffsetName);
      avoidance_direction_offset_ = kAvoidanceDirectionOffsetDefault;
    }
  }

  // Obstacle interference prediction
  std::unique_ptr<CollisionEstimator<Data>> estimator_;
  // Obstacle interference score threshold
  double collision_score_threshold_;
  // Velocity direction search range [rad]
  double search_direction_range_;
  // Previous correction direction [rad]
  double last_adjusted_direction_;
  // Preferred avoidance direction offset [rad]
  double avoidance_direction_offset_;
};

}  // namespace tmc_base_velocity_adjuster

#endif  // TMC_BASE_VELOCITY_ADJUSTER_BASE_VELOCITY_OPTIMIZER_HPP_
