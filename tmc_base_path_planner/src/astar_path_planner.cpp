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
#include <tmc_base_path_planner/astar_path_planner.hpp>

#include <memory>
#include <optional>
#include <vector>

#include <tmc_base_path_planner/common.hpp>

namespace tmc_base_path_planner {
/// Constructor
AstarPathPlanner::AstarPathPlanner(
    const LayeredCostMap::Ptr& map,
    const std::vector<ICostCorrector::Ptr>& cost_correctors) :
    map_(map), cost_correctors_(cost_correctors) {
  astar_executer_ = std::make_shared<AstarExecuter>(AstarExecuter(map->width(), map->height()));
}

/// Execute route planning
bool AstarPathPlanner::PlanPath(
    const Pose2d& start, const Pose2d& goal,
    const CostMapPtr& dynamic_map, const Pose2d& dynamic_map_origin,
    const bool enable_adaptive_start_positioning,
    const PoseSeq& preferred_path,
    PoseSeq& output_path) {
  map_->SetDynamicMap(dynamic_map, dynamic_map_origin);

  // Adjust the starting position
  Pose2d adjusted_start = start;
  if (enable_adaptive_start_positioning) {
    if (!SearchPassablePoint_(map_, start, adjusted_start)) {
      return false;
    }
  }

  // Convert start and goal to grid coordinates
  MapIndex adjusted_start_index;
  MapIndex goal_index;

  map_->PoseToIndex(adjusted_start, adjusted_start_index);
  map_->PoseToIndex(goal, goal_index);

  // If start and goal are in the same grid, return the two points as is
  if (adjusted_start_index == goal_index) {
    output_path.clear();
    output_path.push_back(adjusted_start);
    output_path.push_back(goal);
    return true;
  }

  // Maximum cost estimation
  const int32_t max_cost = map_->EstimateMaxCost(adjusted_start, goal);

  std::vector<MapIndex> preferred_path_indexes;
  PoseSeqToMapIndexes_(preferred_path, preferred_path_indexes);
  // Initialize correction process
  const ICostCorrector::SetupParams setup_params(map_->width(), map_->height(), preferred_path_indexes);
  for (auto& cost_corrector : cost_correctors_) {
    cost_corrector->Setup(setup_params);
  }
  map_->SetCostCollectors(cost_correctors_);

  // Execute pathfinding
  const bool result = astar_executer_->ExecuteAstar(map_, adjusted_start_index, goal_index, max_cost, output_path);
  if (result) {
    // Replace the goal with the input one
    output_path.back() = goal;
  }
  return result;
}

/// Find passable coordinates around the specified point
bool AstarPathPlanner::SearchPassablePoint_(
    const LayeredCostMap::ConstPtr& map, const Pose2d& center, Pose2d& output) const {
  std::vector<MapIndex> indexes;
  map->PoseToIndexes(center, kRangeAdaptiveStartPositioning, indexes);
  std::optional<double> min_distance;
  for (const MapIndex index : indexes) {
    if (map->IsPassable(index)) {
      Pose2d pose;
      map->IndexToPose(index, pose);
      const double distance = (center.point() - pose.point()).norm();
      if (!min_distance || min_distance.value() > distance) {
        min_distance = distance;
        output = pose;
      }
    }
  }
  if (min_distance) {
    return true;
  }
  return false;
}

// Convert from PoseSeq to MapIndex array
void AstarPathPlanner::PoseSeqToMapIndexes_(const PoseSeq& pose_seq, std::vector<MapIndex>& map_indexes) const {
  map_indexes.resize(pose_seq.size());
  for (size_t i = 0; i < pose_seq.size(); ++i) {
    MapIndex index;
    map_->PoseToIndex(pose_seq[i], index);
    map_indexes[i] = index;
  }
}
}  // namespace tmc_base_path_planner
