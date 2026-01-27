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
#include <tmc_base_path_follower/goal_checker_diff_drive.hpp>

#include <angles/angles.h>

namespace {
// Margin to prevent being judged as out of the goal area for slight deviations once entered
// Multiplied with the distance parameter for goal area judgment
constexpr double kGoalAreaMergin = 1.5;
}  // anonymous namespace

namespace tmc_base_path_follower {

// Execute goal check
void DiffDriveGoalChecker::CheckGoal(const PoseSeq& path, const Pose2d& global_pose,
    bool& is_arrived_goal_area, bool& is_arrived_goal) {
  is_arrived_goal = CheckArrivedGoal(path, global_pose);
  is_arrived_goal_area = DiffDriveGoalChecker::CheckArrivedGoalArea(path, global_pose);
  // Retain whether it entered the goal area until the last time
  prev_arrived_goal_area_ = is_arrived_goal_area;
}

// Goal arrival judgment
bool DiffDriveGoalChecker::CheckArrivedGoal(const PoseSeq& path, const Pose2d& global_pose) {
  // If it was in the goal area until the last time and the angle is within the threshold, it is judged to have reached the goal
  if (prev_arrived_goal_area_ &&
      CheckAngleToGoal(path.back(), global_pose, param_.goal_stop_error_angle)) {
    return true;
  }
  return false;
}


// Goal area arrival judgment
// Divide the circular area centered on the goal with a goal line considering the path direction
// Considered as having reached the goal area if it enters the area beyond the goal line
bool DiffDriveGoalChecker::CheckArrivedGoalArea(const PoseSeq& path, const Pose2d& global_pose) {
  double goal_area_length = param_.goal_area_length;
  double goal_line_length = param_.goal_line_length;
  if (prev_arrived_goal_area_) {
    // Apply margin to the judgment value if previously judged as within the goal area
    goal_area_length = goal_area_length * kGoalAreaMergin;
    goal_line_length = goal_line_length * kGoalAreaMergin;
  }
  return IGoalChecker::CheckArrivedGoalArea(path, global_pose, goal_area_length, goal_line_length);
}
}  // namespace tmc_base_path_follower
