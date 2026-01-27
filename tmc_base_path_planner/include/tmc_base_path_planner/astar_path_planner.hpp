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
#ifndef TMC_BASE_PATH_PLANNER_ASTAR_PATH_PLANNER_HPP_
#define TMC_BASE_PATH_PLANNER_ASTAR_PATH_PLANNER_HPP_

#include <vector>
#include <tmc_astar_lib/astar_executer.hpp>
#include <tmc_astar_lib/layered_cost_map.hpp>
#include "path_planner_core.hpp"

using tmc_astar_lib::AstarExecuter;
using tmc_astar_lib::LayeredCostMap;
using tmc_astar_lib::MapIndex;
using tmc_astar_lib::ICostCorrector;

namespace tmc_base_path_planner {

/*
The A* algorithm generates the optimal path from the start point to the goal point.

・The input and output coordinate system is based on the origin of the static map.
・The output goal point matches the input goal.
・Points other than the goal are the coordinates of the center of the grid passed through.
・Except for the goal point, the direction of each point is undefined.

1. Adjust the position of the start point if necessary.
2. If the start and goal exist in the same grid, return them as a two-point path as is.
3. Estimate the maximum cost.
4. Execute the A* algorithm.
   Refer to the tmc_astar_lib package for details.
5. Format the search results into the PoseSeq format.
*/
class AstarPathPlanner : public IPathPlannerCore {
 public:
  /// Constructor
  AstarPathPlanner(
      const LayeredCostMap::Ptr& map,
      const std::vector<ICostCorrector::Ptr>& cost_correctors);
  /// Destructor
  ~AstarPathPlanner() {}
  /// Execute path planning
  /// @param [I] start Start coordinates (relative to static map)
  /// @param [I] goal Goal coordinates (relative to static map)
  /// @param [I] dynamic_map Dynamic map
  /// @param [I] dynamic_map_origin Origin of dynamic map (relative to static map)
  /// @param [I] enable_adaptive_start_positioning Start position adjustment feature ON/OFF
  /// @param [I] preferred_path Preferred path
  /// @param [O] output_path Generated path
  /// @return true: success false: failure
  bool PlanPath(
      const Pose2d& start,
      const Pose2d& goal,
      const CostMapPtr& dynamic_map, const Pose2d& dynamic_map_origin,
      const bool enable_adaptive_start_positioning,
      const PoseSeq& preferred_path,
      PoseSeq& output_path);
  uint8_t static_map_occupancy_threshold() { return map_->static_map_occupancy_threshold(); }

 private:
  // Search for passable coordinates around the specified point
  bool SearchPassablePoint_(const LayeredCostMap::ConstPtr& map, const Pose2d& center, Pose2d& output) const;
  // Convert from PoseSeq to MapIndex array
  void PoseSeqToMapIndexes_(const PoseSeq& pose_seq, std::vector<MapIndex>& map_indexes) const;
  // Astar executer
  AstarExecuter::Ptr astar_executer_;
  // Map
  LayeredCostMap::Ptr map_;
  // Cost adjustment
  const std::vector<ICostCorrector::Ptr> cost_correctors_;
};

}  // namespace tmc_base_path_planner

#endif  // TMC_BASE_PATH_PLANNER_ASTAR_PATH_PLANNER_HPP_
