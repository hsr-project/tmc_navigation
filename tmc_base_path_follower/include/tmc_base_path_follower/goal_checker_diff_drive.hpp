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
#ifndef TMC_BASE_PATH_FOLLOWER_GOAL_CHECKER_DIFF_DRIVE_HPP_
#define TMC_BASE_PATH_FOLLOWER_GOAL_CHECKER_DIFF_DRIVE_HPP_
#include <memory>

#include <console_bridge/console.h>
#include "common.hpp"
#include "goal_checker.hpp"
#include "parameter_default_value.hpp"

namespace tmc_base_path_follower {

/// Goal judgment class for differential two-wheel model
class DiffDriveGoalChecker : public IGoalChecker {
 public:
  /// Parameters
  struct Parameter {
    Parameter(const double in_goal_area_length,
              const double in_goal_line_length,
              const double in_goal_stop_error_angle)
        : goal_area_length(in_goal_area_length),
          goal_line_length(in_goal_line_length),
          goal_stop_error_angle(in_goal_stop_error_angle) {
      if (goal_area_length <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'goal_area_length' is invalid. Use default value.");
        goal_area_length = kGoalAreaLengthDefault;
      }
      if (goal_line_length <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'goal_line_length' is invalid. Use default value.");
        goal_line_length = kGoalLineLengthDefault;
      }
      if (goal_stop_error_angle <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'goal_stop_error_angle' is invalid. Use default value.");
        goal_stop_error_angle = kGoalStopErrorAngleDefault;
      }
    }
    // Distance to goal area
    double goal_area_length;
    // Distance from the goal to the goal area arrival judgment line
    double goal_line_length;
    // Angle error threshold considered as goal
    double goal_stop_error_angle;
  };

  /// Constructor
  /// @param [I] param Parameters
  explicit DiffDriveGoalChecker(const Parameter& param) : prev_arrived_goal_area_(false), param_(param) {}

  /// Initialization
  void Initialize() { prev_arrived_goal_area_ = false; }

  /// Goal judgment execution
  /// @param[I] path Path
  /// @param[I] global_pose Self-position
  /// @param[O] is_arrived_goal_area Goal area arrival judgment
  /// @param[O] is_arrived_goal Goal arrival judgment. If the goal area was not reached during the previous judgment, it will always be false.
  void CheckGoal(const PoseSeq& path, const Pose2d& global_pose, bool& is_arrived_goal_area, bool& is_arrived_goal);

 private:
  /// Goal arrival judgment
  /// @param[I] path Path
  /// @param[I] global_pose Self-position
  bool CheckArrivedGoal(const PoseSeq& path, const Pose2d& global_pose);

  /// Goal area arrival judgment
  /// @param[I] path Path
  /// @param[I] global_pose Self-position
  bool CheckArrivedGoalArea(const PoseSeq& path, const Pose2d& global_pose);

  // Whether the goal area was reached last time
  bool prev_arrived_goal_area_;
  // Parameters
  Parameter param_;
};

}  // namespace tmc_base_path_follower

#endif  // TMC_BASE_PATH_FOLLOWER_GOAL_CHECKER_DIFF_DRIVE_HPP_
