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
#ifndef TMC_BASE_PATH_PLANNER_MAP_FILTER_HPP_
#define TMC_BASE_PATH_PLANNER_MAP_FILTER_HPP_
#include <memory>

#include <console_bridge/console.h>
#include "common.hpp"

namespace tmc_base_path_planner {

class IMapFilter {
 public:
  using Ptr = std::shared_ptr<IMapFilter>;
  virtual ~IMapFilter() = default;
  virtual void FilterMapOnStartAndGoal(
      CostMapPtr& map, const Pose2d& map_origin,
      const Pose2d& start_pose, const Pose2d& goal_pose, const Pose2d& global_pose) = 0;
};

/// Map Filter Class
class MapFilter : public IMapFilter {
 public:
  /// Parameters
  struct Parameter {
    Parameter(const double in_map_filter_range_around_start,
              const double in_map_filter_range_around_goal,
              const double in_map_filter_distance_goal_limit)
        : map_filter_range_around_start(in_map_filter_range_around_start),
          map_filter_range_around_goal(in_map_filter_range_around_goal),
          map_filter_distance_goal_limit(in_map_filter_distance_goal_limit) {
      if (map_filter_range_around_start < 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of '%s' is invalid. Use default value.", kMapFilterRangeAroundStartName);
        map_filter_range_around_start = kMapFilterRangeAroundStartDefault;
      }
      if (map_filter_range_around_goal < 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of '%s' is invalid. Use default value.", kMapFilterRangeAroundGoalName);
        map_filter_range_around_goal = kMapFilterRangeAroundGoalDefault;
      }
      if (map_filter_distance_goal_limit < 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of '%s' is invalid. Use default value.", kMapFilterDistanceGoalLimitName);
        map_filter_distance_goal_limit = kMapFilterDistanceGoalLimitDefault;
      }
    }
    // Range [m] to apply a restricted area filter around the start
    double map_filter_range_around_start;
    // Range [m] to apply a restricted area filter around the goal
    double map_filter_range_around_goal;
    // If the distance from the goal is greater than this, apply a restricted area filter around the goal [m]
    double map_filter_distance_goal_limit;
  };

  /// Constructor
  /// @param [I] param Parameters
  explicit MapFilter(const Parameter& param) : param_(param) {}

  /// Filters obstacles around the start and goal from the map
  /// @param [I/O] map Map
  /// @param [I] map_origin Origin coordinates of the map
  /// @param [I] start_pose Start
  /// @param [I] goal_pose Goal
  /// @param [I] global_pose Self-position
  void FilterMapOnStartAndGoal(CostMapPtr& map, const Pose2d& map_origin,
                               const Pose2d& start_pose, const Pose2d& goal_pose, const Pose2d& global_pose);

 private:
  /// Removes obstacles in a circular area from the map
  /// @param [I/O] map Map
  /// @param [I] center Center point of the circle to be removed
  /// @param [I] radius Radius of the circle to be removed
  void RemoveObstacleInCircle_(CostMapPtr& map, const Pose2d& center, const double radius);

  // Parameters
  Parameter param_;
};
}  // namespace tmc_base_path_planner
#endif  // TMC_BASE_PATH_PLANNER_MAP_FILTER_HPP_
