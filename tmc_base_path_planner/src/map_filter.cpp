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
#include <tmc_base_path_planner/map_filter.hpp>

namespace tmc_base_path_planner {

/// Filter obstacles around the start and goal from the map
void MapFilter::FilterMapOnStartAndGoal(
    CostMapPtr& map, const Pose2d& map_origin,
    const Pose2d& start_pose, const Pose2d& goal_pose, const Pose2d& global_pose) {
  // Filter around the start point
  if (param_.map_filter_range_around_start > 0.0) {
    const double distance_robot_to_start = (global_pose.point() - start_pose.point()).norm();
    const double distance_start_to_goal = (start_pose.point() - goal_pose.point()).norm();
    /// To reduce load, filter only while the start point is within the filter range
    /// If the goal is within the filter range and filtering occurs, it may create a path that collides with obstacles
    /// Filter only when the goal point is outside the filter range
    if (distance_robot_to_start <= param_.map_filter_range_around_start &&
        distance_start_to_goal > param_.map_filter_range_around_start) {
      /// Remove obstacles around the start point
      const Pose2d start_pose_on_map = map_origin.Inverse() * start_pose;
      RemoveObstacleInCircle_(map, start_pose_on_map, param_.map_filter_range_around_start);
    }
  }

  // Filter around the goal point
  if (param_.map_filter_range_around_goal > 0.0) {
    const double distance_robot_to_goal = (global_pose.point() - goal_pose.point()).norm();
    /// Filter only while the distance to the goal is far
    if (distance_robot_to_goal > param_.map_filter_distance_goal_limit) {
      /// Remove obstacles around the goal point
      const Pose2d goal_pose_on_map = map_origin.Inverse() * goal_pose;
      RemoveObstacleInCircle_(map, goal_pose_on_map, param_.map_filter_range_around_goal);
    }
  }
}

/// Remove obstacles in a circular shape from the map
void MapFilter::RemoveObstacleInCircle_(CostMapPtr& map, const Pose2d& center, const double radius) {
  const double radius_square = radius * radius;
  for (uint32_t ih = 0; ih < map->height(); ++ih) {
    const double y = ih * map->resolution();
    for (uint32_t iw = 0; iw < map->width(); ++iw) {
      const double x = iw * map->resolution();
      // Set to FREE if within range
      const double distance_square = (center.point() - Point2d(x, y)).norm_square();
      if (distance_square < radius_square) {
        map->SetValueAt(iw, ih, kFreeGrid);
      }
    }
  }
}
}  // namespace tmc_base_path_planner
