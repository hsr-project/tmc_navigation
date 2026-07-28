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
#ifndef TMC_BASE_PATH_PLANNER_CONDITION_CHECKER_HPP_
#define TMC_BASE_PATH_PLANNER_CONDITION_CHECKER_HPP_
#include <memory>

#include "common.hpp"

namespace tmc_base_path_planner {

class IConditionChecker {
 public:
  using Ptr = std::shared_ptr<IConditionChecker>;
  virtual ~IConditionChecker() = default;
  virtual BasePathPlannerErrorCode CheckCondition(
      const CostMapPtr& static_map, const unsigned char static_map_occupancy_threshold,
      const CostMapPtr& dynamic_map, const Pose2d& dynamic_map_origin,
      const Pose2d& start_pose, const Pose2d& goal_pose, const Pose2d& global_pose) = 0;
};

/// Situation Check Class
class ConditionChecker : public IConditionChecker {
 public:
  /// Constructor
  ConditionChecker() = default;

  /// Check if route planning is possible
  /// @param [I] static_map Static Map
  /// @param [I] static_map_occupancy_threshold Static Map Occupancy Threshold
  /// @param [I] dynamic_map Dynamic Map
  /// @param [I] dynamic_map_origin Origin Pose of Dynamic Map
  /// @param [I] start_pose Start
  /// @param [I] goal_pose Goal
  /// @param [I] global_pose Self Position
  /// @return Error Code
  BasePathPlannerErrorCode CheckCondition(
      const CostMapPtr& static_map, const unsigned char static_map_occupancy_threshold,
      const CostMapPtr& dynamic_map, const Pose2d& dynamic_map_origin,
      const Pose2d& start_pose, const Pose2d& goal_pose, const Pose2d& global_pose);

 private:
  // Check if Pose is within Map boundaries
  bool CheckPoseInMap_(const CostMapPtr& map, const Pose2d& pose);

  // Check if Pose is on Map walls
  bool CheckPoseIsOnMapWall_(const CostMapPtr& map, const unsigned char wall_threshold, const Pose2d& pose);
};
}  // namespace tmc_base_path_planner
#endif  // TMC_BASE_PATH_PLANNER_CONDITION_CHECKER_HPP_
