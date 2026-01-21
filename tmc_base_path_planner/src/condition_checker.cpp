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
#include <tmc_base_path_planner/condition_checker.hpp>

#include <limits>

namespace tmc_base_path_planner {

/// Condition check
BasePathPlannerErrorCode ConditionChecker::CheckCondition(
    const CostMapPtr& static_map, const unsigned char static_map_occupancy_threshold,
    const CostMapPtr& dynamic_map, const Pose2d& dynamic_map_origin,
    const Pose2d& start_pose, const Pose2d& goal_pose, const Pose2d& global_pose) {

  /// Cannot plan a path from outside the static map range
  /// If the robot's position is outside the static map range, it is considered an error
  if (!CheckPoseInMap_(static_map, global_pose)) {
    return BasePathPlannerErrorCode::kRobotIsOutOfMap;
  }

  /// Cannot plan a path outside the static map range
  /// Cannot reach prohibited areas on the static map
  /// If the goal position is outside the static map range or on a prohibited area, it is considered an error
  if (!CheckPoseInMap_(static_map, goal_pose) ||
      CheckPoseIsOnMapWall_(static_map, static_map_occupancy_threshold, goal_pose)) {
    return BasePathPlannerErrorCode::kGoalIsOnStaticObstacle;
  }

  /// Cannot reach prohibited areas on the dynamic map
  /// If the goal position is on a prohibited area of the dynamic map, it is considered an error
  const Pose2d goal_pose_on_dynamic_map = dynamic_map_origin.Inverse() * goal_pose;
  if (CheckPoseIsOnMapWall_(dynamic_map, kWallValue, goal_pose_on_dynamic_map)) {
    return BasePathPlannerErrorCode::kGoalIsOnDynamicObstacle;
  }

  /// Cannot plan a path from on top of an obstacle
  /// If start position adjustment is OFF, it is considered an error if the robot's position is on top of an obstacle
  /// If start position adjustment is ON, the path planning core will attempt to plan from an adjusted position, so it is not considered an error
  if (!kEnableAdaptiveStartPositioning) {
    if (CheckPoseIsOnMapWall_(static_map, static_map_occupancy_threshold, global_pose)) {
      // Error if self-position is on a prohibited area of the static map
      return BasePathPlannerErrorCode::kRobotIsOnStaticObstacle;
    }
    const Pose2d global_pose_on_dynamic_map = dynamic_map_origin.Inverse() * global_pose;
    if (CheckPoseIsOnMapWall_(dynamic_map, kWallValue, global_pose_on_dynamic_map)) {
      // Error if self-position is on a prohibited area of the dynamic map
      return BasePathPlannerErrorCode::kRobotIsOnDynamicObstacle;
    }
  }
  return BasePathPlannerErrorCode::kSuccess;
}

/// Check if the pose is within the map range
bool ConditionChecker::CheckPoseInMap_(const CostMapPtr& map, const Pose2d& pose) {
  Pose2d map_pose = pose;
  map->ImageToMap(map_pose);
  if (!map->InMap(map_pose.point())) {
    return false;
  }
  return true;
}

/// Check if the pose is on the map wall
bool ConditionChecker::CheckPoseIsOnMapWall_(const CostMapPtr& map, const unsigned char wall_threshold,
                                             const Pose2d& pose) {
  const int32_t u = static_cast<int32_t>(pose.x() / map->resolution());
  const int32_t v = static_cast<int32_t>(pose.y() / map->resolution());
  unsigned char value;
  if (!map->GetValueAt(u, v, value) || value < wall_threshold) {
    return false;
  }
  return true;
}
}  // namespace tmc_base_path_planner
