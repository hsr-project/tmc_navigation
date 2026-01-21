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
#include <tmc_base_path_planner/base_path_planner.hpp>

#include <vector>

namespace tmc_base_path_planner {

/// Constructor
BasePathPlanner::BasePathPlanner(
    const CostMapPtr& static_map,
    const IConditionChecker::Ptr& condition_checker,
    const IMapFilter::Ptr& map_filter,
    const IPathUpdater::Ptr& path_updater,
    const IPathSmoother::Ptr& path_smoother,
    const IPathPlannerCore::Ptr& planner_core)
    : IBasePathPlanner(),
      planner_core_(planner_core),
      static_map_(static_map),
      condition_checker_(condition_checker),
      map_filter_(map_filter),
      path_updater_(path_updater),
      path_smoother_(path_smoother) {}

/// Initialization
void BasePathPlanner::Initialize() {
  planned_grid_path_.clear();
  path_updater_->ClearPrevPath();
}

/// Path Planning
BasePathPlannerErrorCode BasePathPlanner::PlanPath(
    const Pose2d& in_start_pose, const Pose2d& in_goal_pose, const Pose2d& in_global_pose,
    const CostMapPtr& in_dynamic_map, const Pose2d& in_dynamic_map_origin, const bool in_skip_non_update,
    PoseSeq& out_path) {
  Pose2d start_pose = in_start_pose;
  Pose2d goal_pose = in_goal_pose;
  Pose2d global_pose = in_global_pose;
  Pose2d dynamic_map_origin = in_dynamic_map_origin;

  // Convert each coordinate from the map frame coordinate system to the static map reference coordinate system
  static_map_->MapToImage(start_pose);
  static_map_->MapToImage(goal_pose);
  static_map_->MapToImage(global_pose);
  static_map_->MapToImage(dynamic_map_origin);
  // Filter obstacles around the start and goal to prevent complete failure in path planning when they are blocked by obstacles
  CostMapPtr dynamic_map = in_dynamic_map;
  map_filter_->FilterMapOnStartAndGoal(dynamic_map, dynamic_map_origin, start_pose, goal_pose, global_pose);

  // Pre-check if path planning is possible
  BasePathPlannerErrorCode error_code = condition_checker_->CheckCondition(
      static_map_, static_map_occupancy_threshold(), dynamic_map, dynamic_map_origin,
      start_pose, goal_pose, global_pose);
  if (error_code != BasePathPlannerErrorCode::kSuccess) {
    path_updater_->ClearPrevPath();
    return error_code;
  }

  // Determine the start position for path planning
  Pose2d plan_start_pose;
  const std::optional<uint32_t> start_index_on_prev_path = path_updater_->SearchStartPoseOnPrevPath(
      global_pose, plan_start_pose);

  // Grid path planning
  if (!planner_core_->PlanPath(plan_start_pose, goal_pose, dynamic_map, dynamic_map_origin,
                               kEnableAdaptiveStartPositioning, planned_grid_path_, planned_grid_path_)) {
    path_updater_->ClearPrevPath();
    return BasePathPlannerErrorCode::kPlanningFail;
  }
  // Update the path
  PoseSeq update_path;
  if (!path_updater_->UpdatePath(planned_grid_path_, start_index_on_prev_path, update_path)) {
    if (in_skip_non_update) {
      /// Skip if no update is needed for the previous path
      return BasePathPlannerErrorCode::kSkip;
    }
  }

  // Smooth the path
  PoseSeq smooth_path;
  if (!path_smoother_->SmoothingPath(update_path, smooth_path)) {
    // Fail if path smoothing fails
    path_updater_->ClearPrevPath();
    return BasePathPlannerErrorCode::kSmoothingFail;
  }

  // Convert from image coordinate system to map coordinate system and output
  for (PoseSeq::iterator it = smooth_path.begin(); it != smooth_path.end(); ++it) {
    Pose2d pose = *it;
    static_map_->ImageToMap(pose);
    out_path.push_back(pose);
  }
  return BasePathPlannerErrorCode::kSuccess;
}
}  // namespace tmc_base_path_planner
