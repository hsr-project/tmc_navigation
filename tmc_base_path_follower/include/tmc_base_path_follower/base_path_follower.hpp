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
/// @file base_path_follower.hpp
/// @brief Cart path-following control class
#ifndef TMC_BASE_PATH_FOLLOWER_BASE_PATH_FOLLOWER_HPP_
#define TMC_BASE_PATH_FOLLOWER_BASE_PATH_FOLLOWER_HPP_
#include <memory>
#include <optional>

#include "common.hpp"
#include "goal_checker.hpp"
#include "nearest_path_point_searcher.hpp"
#include "path_transit_velocity_calculator.hpp"
#include "velocity_calculator.hpp"

namespace tmc_base_path_follower {

// Cart path-following control class
class BasePathFollower {
 public:
  using Ptr = std::shared_ptr<BasePathFollower>;
  /// Constructor
  /// @param [I] nearest_path_point_searcher Path nearest point search functionality
  /// @param [I] goal_checker Goal determination functionality
  /// @param [I] velocity_calculator Velocity calculation functionality
  /// @param [I] path_transit_velocity_calculator Path transit velocity calculation functionality (specify nullptr if not needed)
  BasePathFollower(
      const INearestPathPointSearcher::Ptr& nearest_path_point_searcher,
      const IGoalChecker::Ptr& goal_checker,
      const IVelocityCalculator::Ptr& velocity_calculator,
      const IPathTransitVelocityCalculator::Ptr& path_transit_velocity_calculator);

  /// Destructor
  ~BasePathFollower() {}

  /// Initialization
  /// @param [I] path_info Path information
  void Initialize(const PathInfo& path_info);

  /// Path-following velocity calculation
  /// @param [I] global_pose Self-position
  /// @param [I] last_velocity Velocity from the previous step
  /// @param [I] time_interval Time interval from the previous step
  /// @param [O] is_arrived_goal Goal arrival determination result
  /// @param [O] current_path_index Current path index
  /// @param [O] output_velocity Output velocity
  /// @return Path-following success or failure
  bool FollowPathVelocity(
      const Pose2d& global_pose, const Vector3d& last_velocity, const double time_interval,
      bool& is_arrived_goal, uint32_t& current_path_index, Vector3d& output_velocity);

 private:
  /// Path nearest point search functionality
  INearestPathPointSearcher::Ptr nearest_path_point_searcher_;
  /// Goal determination functionality
  IGoalChecker::Ptr goal_checker_;
  /// Velocity calculation functionality
  IVelocityCalculator::Ptr velocity_calculator_;
  /// Path transit velocity calculation functionality
  IPathTransitVelocityCalculator::Ptr path_transit_velocity_calculator_;
  /// Path being followed
  PathInfo path_info_;
  /// Current path index
  std::optional<uint32_t> current_path_index_;
};

}  // namespace tmc_base_path_follower
#endif /*TMC_BASE_PATH_FOLLOWER_BASE_PATH_FOLLOWER_HPP_*/
