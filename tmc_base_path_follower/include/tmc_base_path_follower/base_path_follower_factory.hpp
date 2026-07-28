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
#ifndef TMC_BASE_PATH_FOLLOWER_BASE_PATH_FOLLOWER_FACTORY_HPP_
#define TMC_BASE_PATH_FOLLOWER_BASE_PATH_FOLLOWER_FACTORY_HPP_
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "base_path_follower.hpp"
#include "parameter_creator.hpp"

namespace tmc_base_path_follower {

/// Create BasePathFollower object
BasePathFollower::Ptr CreateBasePathFollower(const rclcpp::Node::SharedPtr& node) {
  // Path proximity point search functionality
  INearestPathPointSearcher::Ptr nearest_path_point_searcher = std::make_shared<NearestPathPointSearcher>(
      CreateNearestPathPointSearcherParameter(node));

  // Generate speed calculation class and goal determination class for the specified speed model
  std::string move_model_name;
  GetOptionalParam(node, "move_model_name", move_model_name, std::string(kMoveModelNameDefault));

  IVelocityCalculator::Ptr velocity_calculator;
  IGoalChecker::Ptr goal_checker;
  if (move_model_name == "diff_drive") {
    velocity_calculator.reset(new DiffDriveVelocityCalculator(
        CreateDiffDriveVelocityCalculatorParameter(node)));
    goal_checker.reset(new DiffDriveGoalChecker(CreateDiffDriveGoalCheckerParameter(node)));
  } else if (move_model_name == "omni") {
    velocity_calculator.reset(new OmniVelocityCalculator(CreateOmniVelocityCalculatorParameter(node)));
    goal_checker.reset(new OmniGoalChecker(CreateOmniGoalCheckerParameter(node)));
  } else {
    throw std::runtime_error("Unknown move model: " + move_model_name);
  }

  // Path traversal speed calculation functionality
  bool use_path_transit_velocity;
  GetOptionalParam(node, "use_path_transit_velocity", use_path_transit_velocity, kUsePathTransitVelocityDefault);

  IPathTransitVelocityCalculator::Ptr path_transit_velocity_calculator;
  if (use_path_transit_velocity) {
    path_transit_velocity_calculator.reset(
      new PathTransitVelocityCalculator(CreatePathTransitVelocityCalculatorParameter(node)));
  } else {
    // Specify nullptr if path traversal speed limit is not required
    path_transit_velocity_calculator = nullptr;
  }
  BasePathFollower::Ptr follower;
  follower.reset(new BasePathFollower(
      nearest_path_point_searcher,
      goal_checker,
      velocity_calculator, path_transit_velocity_calculator));
  return follower;
}

}  // namespace tmc_base_path_follower
#endif /*TMC_BASE_PATH_FOLLOWER_BASE_PATH_FOLLOWER_FACTORY_HPP_*/
