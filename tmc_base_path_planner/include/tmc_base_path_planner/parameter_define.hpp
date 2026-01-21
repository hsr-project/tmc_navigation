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
/// parameter_define.hpp
#ifndef TMC_BASE_PATH_PLANNER_PARAMETER_DEFINE_HPP_
#define TMC_BASE_PATH_PLANNER_PARAMETER_DEFINE_HPP_

namespace tmc_base_path_planner {
/// Parameter Definition
// BasePathPlannerNode Space
constexpr const char* const kBasePathPlannerNodeSpace = "node";
// Drive Cycle [Hz]
constexpr const char* const kRateName = "rate";
constexpr double kRateDefault = 10.0;
// Dynamic Map Timeout [s]
constexpr const char* const kDynamicMapTimeoutName = "dynamic_map_timeout";
constexpr double kDynamicMapTimeoutDefault = 1.0;
// Self-Position Timeout [s]
constexpr const char* const kGlobalPoseTimeoutName = "global_pose_timeout";
constexpr double kGlobalPoseTimeoutDefault = 2.0;
// Distance to Expand Obstacle Area from Wall in Static Map [m]
constexpr const char* const kStaticMapPotentialWidthName = "static_map_potential_width";
constexpr double kStaticMapPotentialWidthDefault = 3.0;

// BasePathPlanner Space
constexpr const char* const kBasePathPlannerSpace = "base_path_planner";
// Planner Type [str]
constexpr const char* const kTypeName = "type";
constexpr const char* const kTypeDefault = "astar_path_planner";

// MapFilter Space
constexpr const char* const kMapFilterSpace = "map_filter";
// Range to Remove Prohibited Area Around Start [m]
constexpr const char* const kMapFilterRangeAroundStartName = "map_filter_range_around_start";
constexpr double kMapFilterRangeAroundStartDefault = 0.0;
// Distance to Goal for Filtering Prohibited Area Around Goal [m]
// Remove Prohibited Area Around Goal When Distance Between Goal and Robot Exceeds This Value
constexpr const char* const kMapFilterDistanceGoalLimitName = "map_filter_distance_goal_limit";
constexpr double kMapFilterDistanceGoalLimitDefault = 1.0;
// Range to Remove Prohibited Area Around Goal [m]
constexpr const char* const kMapFilterRangeAroundGoalName = "map_filter_range_around_goal";
constexpr double kMapFilterRangeAroundGoalDefault = 0.5;

// PathUpdater Space
constexpr const char* const kPathUpdaterSpace = "path_updater";
// Distance to Previous Path to Consider Being on Previous Path [m]
constexpr const char* const kDistanceOnPrevPathName = "distance_on_prev_path";
constexpr double kDistanceOnPrevPathDefault = 0.15;
// Allowable Deviation Between Route Points [m]
constexpr const char* const kGridErrorName = "grid_error";
constexpr double kGridErrorDefault = 0.1;
// Threshold for Merging Previous Path if Points Match Up to This Number [num]
constexpr const char* const kSamePointNumMergePathName = "same_point_num_merge_path";
constexpr int32_t kSamePointNumMergePathDefault = 3;

// AstarPathPlanner Space
constexpr const char* const kAstarPathPlannerSpace = "astar_path_planner";
// Prohibited Area [m]
constexpr const char* const kExclusiveSizeName = "exclusive_size";
constexpr double kExclusiveSizeDefault = 0.2;
// Potential Area [m]
constexpr const char* const kPotentialSizeName = "potential_size";
constexpr double kPotentialSizeDefault = 0.07;
// Maximum Cost Estimate Ratio for Distance Between Start and Goal
constexpr const char* const kCostFactorName = "cost_factor";
constexpr double kCostFactorDefault = 150.0;
// Cost Value Indicating Unknown Area
constexpr const char* const kCostUnknownName = "cost_unknown";
constexpr int32_t kCostUnknownDefault = 255;

// Movement Cost Between Adjacent Grids
constexpr const char* const kSingleCostName = "single_cost";
constexpr int32_t kSingleCostDefault = 50;
// Movement Cost for Diagonal Grids
constexpr const char* const kDiagonalCostName = "diagonal_cost";
constexpr int32_t kDiagonalCostDefault = 71;

// Cost Correction Value for Grids on Recommended Path
constexpr const char* const kCostOnPreferredPathName = "cost_on_preferred_path";
constexpr int32_t kCostOnPreferredPathDefault = 0;

// Cost Correction Value for Grids Around Recommended Path
constexpr const char* const kCostAroundPreferredPathName = "cost_around_preferred_path";
constexpr int32_t kCostAroundPreferredPathDefault = 0;
}  // namespace tmc_base_path_planner
#endif  // TMC_BASE_PATH_PLANNER_PLANNER_PARAMETERS_HPP_
