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
#ifndef TMC_BASE_PATH_PLANNER_BASE_PATH_PLANNER_HPP_
#define TMC_BASE_PATH_PLANNER_BASE_PATH_PLANNER_HPP_

#include <memory>
#include "common.hpp"
#include "condition_checker.hpp"
#include "map_filter.hpp"
#include "path_planner_core.hpp"
#include "path_smoother.hpp"
#include "path_updater.hpp"

namespace tmc_base_path_planner {

/// Route Planning Base Class
class IBasePathPlanner {
 public:
  using Ptr = std::shared_ptr<IBasePathPlanner>;
  /// Constructor
  IBasePathPlanner() = default;
  virtual ~IBasePathPlanner() = default;

  /// Initialization
  virtual void Initialize() = 0;

  /// Plan Route
  /// @param [I] in_start_pose Start
  /// @param [I] in_goal_pose Goal
  /// @param [I] in_global_pose Self Position
  /// @param [I] in_dynamic_map Dynamic Map
  /// @param [I] in_dynamic_map_origin Dynamic Map Origin Pose
  /// @param [I] in_skip_non_update Whether to skip if there is no update to the previous route
  /// @param [O] out_path Planned Route
  virtual BasePathPlannerErrorCode PlanPath(
      const Pose2d& in_start_pose, const Pose2d& in_goal_pose, const Pose2d& in_global_pose,
      const CostMapPtr& in_dynamic_map, const Pose2d& in_dynamic_map_origin, const bool in_skip_non_update,
      PoseSeq& out_path) = 0;
  virtual uint8_t static_map_occupancy_threshold() const = 0;
};

/// Route Planning Class
class BasePathPlanner : public IBasePathPlanner {
 public:
  /// Constructor
  /// @param [I] static_map Static Map
  /// @param [I] condition_checker Condition Checker
  /// @param [I] map_filter Map Filter
  /// @param [I] path_updater Path Updater
  /// @param [I] path_smoother Path Smoother
  /// @param [I] planner_core Route Planning Core
  BasePathPlanner(
      const CostMapPtr& static_map,
      const IConditionChecker::Ptr& condition_checker,
      const IMapFilter::Ptr& map_filter,
      const IPathUpdater::Ptr& path_updater,
      const IPathSmoother::Ptr& path_smoother,
      const IPathPlannerCore::Ptr& planner_core);

  void Initialize();

  /// Plan Route
  /// @param [I] in_start_pose Start
  /// @param [I] in_goal_pose Goal
  /// @param [I] in_global_pose Self Position
  /// @param [I] in_dynamic_map Dynamic Map
  /// @param [I] in_dynamic_map_origin Dynamic Map Origin Pose
  /// @param [I] in_skip_non_update Whether to skip if there is no update to the previous route
  /// @param [O] out_path Planned Route
  BasePathPlannerErrorCode PlanPath(
      const Pose2d& in_start_pose, const Pose2d& in_goal_pose, const Pose2d& in_global_pose,
      const CostMapPtr& in_dynamic_map, const Pose2d& in_dynamic_map_origin, const bool in_skip_non_update,
      PoseSeq& out_path);

  uint8_t static_map_occupancy_threshold() const { return planner_core_->static_map_occupancy_threshold(); }

 private:
  // Route Planning Core
  IPathPlannerCore::Ptr planner_core_;
  // Static Map
  CostMapPtr static_map_;
  // Condition Checker
  IConditionChecker::Ptr condition_checker_;
  // Map Filter
  IMapFilter::Ptr map_filter_;
  // Path Updater
  IPathUpdater::Ptr path_updater_;
  // Route Smoothing
  IPathSmoother::Ptr path_smoother_;
  // Previous Route
  PoseSeq planned_grid_path_;
};

}  // namespace tmc_base_path_planner

#endif  // TMC_BASE_PATH_PLANNER_BASE_PATH_PLANNER_HPP_
