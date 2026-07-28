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
/// @file base_path_follower.cpp
/// @brief Cart path-following control class
#include <tmc_base_path_follower/base_path_follower.hpp>

#include <optional>

namespace tmc_base_path_follower {

/// Constructor
/// @param [I] nearest_path_point_searcher Path nearest point search functionality
/// @param [I] goal_checker Goal determination functionality
/// @param [I] velocity_calculator Velocity calculation functionality
/// @param [I] path_transit_velocity_calculator Path transit velocity calculation functionality (specify nullptr if not needed)
BasePathFollower::BasePathFollower(
    const INearestPathPointSearcher::Ptr& nearest_path_point_searcher,
    const IGoalChecker::Ptr& goal_checker,
    const IVelocityCalculator::Ptr& velocity_calculator,
    const IPathTransitVelocityCalculator::Ptr& path_transit_velocity_calculator)
    : nearest_path_point_searcher_(nearest_path_point_searcher),
      goal_checker_(goal_checker), velocity_calculator_(velocity_calculator),
      path_transit_velocity_calculator_(path_transit_velocity_calculator) {}

/// Initialization
/// @param [I] path_info Path information
void BasePathFollower::Initialize(const PathInfo& path_info) {
  path_info_ = path_info;
  if (path_transit_velocity_calculator_ != nullptr) {
    // Generate transit velocity information based on the path
    path_transit_velocity_calculator_->CalculatePathTransitVelocity(path_info);
  }
  current_path_index_ = std::nullopt;
  goal_checker_->Initialize();
}

/// Path-following velocity calculation
/// @param [I] global_pose Self-position
/// @param [I] last_velocity Velocity from the previous step
/// @param [I] time_interval Time interval from the previous step
/// @param [O] is_arrived_goal Goal arrival determination result
/// @param [O] current_path_index Current path index
/// @param [O] output_velocity Output velocity
/// @return Path-following success or failure
bool BasePathFollower::FollowPathVelocity(
    const Pose2d& global_pose, const Vector3d& last_velocity,
    const double time_interval, bool& is_arrived_goal, uint32_t& current_path_index,
    Vector3d& output_velocity) {
  // Search for the nearest point
  current_path_index_ = nearest_path_point_searcher_->SearchNearestPathPointIndex(
      path_info_.splined_path, path_info_.splined_path_left_lengths, global_pose, current_path_index_);
  current_path_index = current_path_index_.value();

  bool is_arrived_goal_area = false;
  // Goal determination
  goal_checker_->CheckGoal(path_info_.splined_path, global_pose, is_arrived_goal_area, is_arrived_goal);

  if (is_arrived_goal) {
    // Return velocity 0 if the goal is reached
    output_velocity = Vector3d::Zero();
    return true;
  }

  std::optional<double> transit_velocity = std::nullopt;
  if (path_transit_velocity_calculator_ != nullptr) {
    // Apply correction based on the transit velocity of the path
    transit_velocity = path_transit_velocity_calculator_->GetPathTransitVelocity(current_path_index);
  }

  // Velocity calculation
  const bool result = velocity_calculator_->CalculateVelocity(
      path_info_, global_pose, current_path_index,
      last_velocity, time_interval, is_arrived_goal_area, transit_velocity, output_velocity);
  if (!result) {
    output_velocity = Vector3d::Zero();
    return false;
  }
  return true;
}
}  // namespace tmc_base_path_follower
