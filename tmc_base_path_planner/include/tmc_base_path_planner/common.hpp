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
#ifndef TMC_BASE_PATH_PLANNER_COMMON_HPP_
#define TMC_BASE_PATH_PLANNER_COMMON_HPP_
#include <memory>
#include <optional>
#include <vector>

#include <tmc_pose_2d_lib/distance_map.hpp>
#include <tmc_pose_2d_lib/pose_2d.hpp>
#include "parameter_define.hpp"

using CostMap = tmc_pose_2d_lib::DistanceMap;
using CostMapPtr = std::shared_ptr<CostMap>;
using tmc_pose_2d_lib::Pose2d;
using tmc_pose_2d_lib::Point2d;

namespace tmc_base_path_planner {
/// Map Free Value
constexpr uint8_t kFreeGrid = 1;
/// Map Wall Value
constexpr uint8_t kWallValue = 255;
// Start Position Adjustment Function On/Off
constexpr bool kEnableAdaptiveStartPositioning = true;
// Range of Start Position Adjustment [m]
constexpr double kRangeAdaptiveStartPositioning = 1.0;

typedef std::vector<Pose2d> PoseSeq;

/// Planner Error Code
enum class BasePathPlannerErrorCode {
  // Route Planning Success
  kSuccess = 1,
  // Skipped because the route is the same as the previous one
  kSkip = 2,
  // Robot position is outside the map range
  kRobotIsOutOfMap = -1,
  // Robot position is occupied by the static map
  kRobotIsOnStaticObstacle = -2,
  // Robot position is occupied by the dynamic map
  kRobotIsOnDynamicObstacle = -3,
  // Goal position is occupied by the static map
  kGoalIsOnStaticObstacle = -4,
  // Goal position is occupied by the dynamic map
  kGoalIsOnDynamicObstacle = -5,
  // Route Planning Process Failed
  kPlanningFail = -6,
  // Route Smoothing Process Failed
  kSmoothingFail = -7,
};

/// Search for the index of the nearest point on the route
/// @param [I] path Route
/// @param [I] pose Position
/// @return Index of the nearest point
std::optional<uint32_t> SearchNearestPointIndexOnPath(const PoseSeq& path, const Pose2d& pose);
/// Calculate the length of the route
/// @param [I] Route
/// @return Length of the route
double CalculatePathLength(const PoseSeq& path);
}  // namespace tmc_base_path_planner

#endif  // TMC_BASE_PATH_PLANNER_COMMON_HPP_
