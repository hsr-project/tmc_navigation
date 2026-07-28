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
/// base_path_planner_factory.cpp
#include <memory>
#include <string>

#include <tmc_base_path_planner/base_path_planner_factory.hpp>
#include <tmc_base_path_planner/param.hpp>

namespace tmc_base_path_planner {

/// BasePathPlanner object creation
template <typename T>
BasePathPlanner::Ptr BasePathPlannerFactory::Create(const T& node,
    const CostMapPtr& static_map, const double static_map_potential_width) {
  std::string planner_type;
  BasePathPlanner::Ptr planner;
  std::map<std::string, rclcpp::Parameter> base_path_planner_params;
  GetGroupParam(node, kBasePathPlannerSpace, base_path_planner_params);
  GetOptionalParam(base_path_planner_params, kTypeName, planner_type, std::string(kTypeDefault));
  if (planner_type == "astar_path_planner") {
    planner.reset(new BasePathPlanner(
        static_map,
        std::make_shared<ConditionChecker>(),
        std::make_shared<MapFilter>(CreateMapFilterParameter(base_path_planner_params)),
        std::make_shared<PathUpdater>(CreatePathUpdaterParameter(base_path_planner_params)),
        std::make_shared<PathSmoother>(),
        AstarPathPlannerFactory::Create(base_path_planner_params, static_map, static_map_potential_width)));
  } else {
    throw std::runtime_error("Unknown planner type: " + planner_type);
  }
  return planner;
}

// Explicit instantiation of template instance
template BasePathPlanner::Ptr BasePathPlannerFactory::Create<rclcpp::Node::SharedPtr>(
    const rclcpp::Node::SharedPtr& node,
    const CostMapPtr& static_map, const double static_map_potential_width);
template BasePathPlanner::Ptr BasePathPlannerFactory::Create<rclcpp_lifecycle::LifecycleNode::SharedPtr>(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const CostMapPtr& static_map, const double static_map_potential_width);

/// MapFilter parameter creation
MapFilter::Parameter BasePathPlannerFactory::CreateMapFilterParameter(
    const std::map<std::string, rclcpp::Parameter> params) {

  std::map<std::string, rclcpp::Parameter> map_filter_params;
  GetGroupParam(params, kMapFilterSpace, map_filter_params);

  double map_filter_range_around_start;
  GetOptionalParam(map_filter_params, kMapFilterRangeAroundStartName, map_filter_range_around_start,
                   kMapFilterRangeAroundStartDefault);
  double map_filter_range_around_goal;
  GetOptionalParam(map_filter_params, kMapFilterRangeAroundGoalName, map_filter_range_around_goal,
                   kMapFilterRangeAroundGoalDefault);
  double map_filter_distance_goal_limit;
  GetOptionalParam(map_filter_params, kMapFilterDistanceGoalLimitName, map_filter_distance_goal_limit,
                   kMapFilterDistanceGoalLimitDefault);
  return MapFilter::Parameter(map_filter_range_around_start, map_filter_range_around_goal,
                              map_filter_distance_goal_limit);
}

/// PathUpdater parameter creation
PathUpdater::Parameter BasePathPlannerFactory::CreatePathUpdaterParameter(
    const std::map<std::string, rclcpp::Parameter> params) {
  std::map<std::string, rclcpp::Parameter> path_updater_params;
  GetGroupParam(params, kPathUpdaterSpace, path_updater_params);

  double distance_on_prev_path;
  GetOptionalParam(path_updater_params, kDistanceOnPrevPathName, distance_on_prev_path,
                   kDistanceOnPrevPathDefault);
  double grid_error;
  GetOptionalParam(path_updater_params, kGridErrorName, grid_error,
                   kGridErrorDefault);
  int32_t same_point_num_merge_path;
  GetOptionalParam(path_updater_params, kSamePointNumMergePathName, same_point_num_merge_path,
                   kSamePointNumMergePathDefault);
  return PathUpdater::Parameter(distance_on_prev_path, grid_error, same_point_num_merge_path);
}
}  // namespace tmc_base_path_planner
