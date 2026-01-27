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
// Utility functions for testing without ROS dependency
#ifndef TMC_BASE_PATH_PLANNER_TEST_UTILS_HPP_
#define TMC_BASE_PATH_PLANNER_TEST_UTILS_HPP_

#include <limits>
#include <vector>

#include <angles/angles.h>

namespace tmc_base_path_planner {
// Check if two points match
bool IsMatchPoses(const Pose2d& pose1, const Pose2d& pose2) {
  if (fabs(pose1.x() - pose2.x()) > std::numeric_limits<double>::epsilon() ||
      fabs(pose1.y() - pose2.y()) > std::numeric_limits<double>::epsilon() ||
      fabs(angles::shortest_angular_distance(pose1.theta(), pose2.theta())) >
      std::numeric_limits<double>::epsilon()) {
    return false;
  }
  return true;
}

// Check if two paths match
bool IsMatchPaths(const PoseSeq& path1, const PoseSeq& path2) {
  if (path1.size() != path2.size()) {
    return false;
  }
  for (uint32_t i = 0; i < path1.size(); ++i) {
    if (!IsMatchPoses(path1[i], path2[i])) {
      return false;
    }
  }
  return true;
}
}  // namespace tmc_base_path_planner
#endif  // TMC_BASE_PATH_PLANNER_TEST_UTILS_HPP_
