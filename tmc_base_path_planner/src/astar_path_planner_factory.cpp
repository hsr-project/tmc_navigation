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
#include <tmc_base_path_planner/astar_path_planner_factory.hpp>

#include <memory>
#include <string>
#include <vector>

#include <tmc_base_path_planner/param.hpp>
#include <tmc_base_path_planner/parameter_define.hpp>

namespace tmc_base_path_planner {

/// Create PathPlannerCore object
IPathPlannerCore::Ptr AstarPathPlannerFactory::Create(
    const std::map<std::string, rclcpp::Parameter> params,
    const CostMapPtr& static_map, const double static_map_potential_width) {

  std::map<std::string, rclcpp::Parameter> astar_path_planner_params;
  GetGroupParam(params, kAstarPathPlannerSpace, astar_path_planner_params);

  // Create an instance of the cost correction class
  std::vector<ICostCorrector::Ptr> cost_correctors;
  cost_correctors.push_back(std::make_shared<DirectionCostCorrector>());
  cost_correctors.push_back(std::make_shared<DynamicObstacleCostCorrector>());

  PreferredPathCostCorrector::Parameter preferred_path_cost_corrector_param =
      CreatePreferredPathCostCorrectorParameter(astar_path_planner_params);
  if (preferred_path_cost_corrector_param.cost_on_preferred_path != 0 ||
      preferred_path_cost_corrector_param.cost_around_preferred_path != 0) {
    // If the correction value is 0, no processing is required, so the instance itself is not created
    cost_correctors.push_back(std::make_shared<PreferredPathCostCorrector>(
      preferred_path_cost_corrector_param));
  }

  // Create an instance of AstarPathPlanner
  IPathPlannerCore::Ptr planner;

  planner.reset(new AstarPathPlanner(
      std::make_shared<LayeredCostMap>(
      LayeredCostMap(CreateLayeredCostMapParameter(astar_path_planner_params, static_map_potential_width),
      static_map)), cost_correctors));
  return planner;
}

/// Generate LayearedCostMap parameters
LayeredCostMap::Parameter AstarPathPlannerFactory::CreateLayeredCostMapParameter(
    const std::map<std::string, rclcpp::Parameter> params, const double static_map_potential_width) {

  double exclusive_size;
  GetOptionalParam(params, kExclusiveSizeName, exclusive_size, kExclusiveSizeDefault);
  double potential_size;
  GetOptionalParam(params, kPotentialSizeName, potential_size, kPotentialSizeDefault);
  double cost_factor;
  GetOptionalParam(params, kCostFactorName, cost_factor, kCostFactorDefault);
  int32_t cost_unknown;
  GetOptionalParam(params, kCostUnknownName, cost_unknown, kCostUnknownDefault);
  int32_t single_cost;
  GetOptionalParam(params, kSingleCostName, single_cost, kSingleCostDefault);
  int32_t diagonal_cost;
  GetOptionalParam(params, kDiagonalCostName, diagonal_cost, kDiagonalCostDefault);
  return LayeredCostMap::Parameter(exclusive_size, potential_size, static_map_potential_width, cost_factor,
                                   cost_unknown, single_cost, diagonal_cost);
}

/// Generate PreferredPathCostCorrector parameters
PreferredPathCostCorrector::Parameter AstarPathPlannerFactory::CreatePreferredPathCostCorrectorParameter(
    const std::map<std::string, rclcpp::Parameter> params) {
  int32_t cost_on_preferred_path;
  GetOptionalParam(params, kCostOnPreferredPathName, cost_on_preferred_path, kCostOnPreferredPathDefault);
  int32_t cost_around_preferred_path;
  GetOptionalParam(params, kCostAroundPreferredPathName, cost_around_preferred_path, kCostAroundPreferredPathDefault);
  return PreferredPathCostCorrector::Parameter(cost_on_preferred_path, cost_around_preferred_path);
}
}  // namespace tmc_base_path_planner
