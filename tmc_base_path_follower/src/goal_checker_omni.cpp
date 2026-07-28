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
#include <tmc_base_path_follower/goal_checker_omni.hpp>

namespace tmc_base_path_follower {

// Goal check execution
void OmniGoalChecker::CheckGoal(const PoseSeq& path, const Pose2d& global_pose,
    bool& is_arrived_goal_area, bool& is_arrived_goal) {
  is_arrived_goal = CheckArrivedGoal(path, global_pose);
  is_arrived_goal_area = CheckArrivedGoalArea(path, global_pose, param_.goal_area_length, param_.goal_line_length);
}

// Goal achievement determination
bool OmniGoalChecker::CheckArrivedGoal(const PoseSeq& path, const Pose2d& global_pose) {
  if (CheckDistanceToGoal(path.back(), global_pose, param_.goal_stop_error_length) &&
      CheckAngleToGoal(path.back(), global_pose, param_.goal_stop_error_angle)) {
    // If both distance and angle are within the threshold, it is determined that the goal has been reached
    return true;
  }
  return false;
}
}  // namespace tmc_base_path_follower
