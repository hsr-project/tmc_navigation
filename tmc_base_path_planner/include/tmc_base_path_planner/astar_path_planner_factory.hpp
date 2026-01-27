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
#ifndef TMC_BASE_PATH_PLANNER_ASTAR_PATH_PLANNER_FACTORY_HPP_
#define TMC_BASE_PATH_PLANNER_ASTAR_PATH_PLANNER_FACTORY_HPP_
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tmc_astar_lib/cost_corrector/cost_corrector.hpp>
#include <tmc_astar_lib/cost_corrector/direction_cost_corrector.hpp>
#include <tmc_astar_lib/cost_corrector/dynamic_obstacle_cost_corrector.hpp>
#include <tmc_astar_lib/cost_corrector/preferred_path_cost_corrector.hpp>

#include "astar_path_planner.hpp"

using tmc_astar_lib::LayeredCostMap;
using tmc_astar_lib::ICostCorrector;
using tmc_astar_lib::PreferredPathCostCorrector;
using tmc_astar_lib::DirectionCostCorrector;
using tmc_astar_lib::DynamicObstacleCostCorrector;
namespace tmc_base_path_planner {

class AstarPathPlannerFactory {
 public:
  /// PathPlannerCore object creation
  static IPathPlannerCore::Ptr Create(const std::map<std::string, rclcpp::Parameter> params,
                                      const CostMapPtr& static_map,
                                      const double static_map_potential_width);

 private:
  /// AstarLibrary parameter creation
  static LayeredCostMap::Parameter CreateLayeredCostMapParameter(
      const std::map<std::string, rclcpp::Parameter> params,
      const double static_map_potential_width);
  static PreferredPathCostCorrector::Parameter CreatePreferredPathCostCorrectorParameter(
      const std::map<std::string, rclcpp::Parameter> params);
};
}  // namespace tmc_base_path_planner

#endif  // TMC_BASE_PATH_PLANNER_ASTAR_PATH_PLANNER_FACTORY_HPP_
