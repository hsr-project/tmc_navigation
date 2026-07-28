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
#include <tmc_base_path_planner/path_updater.hpp>

#include <limits>

namespace tmc_base_path_planner {

/// Clear the previous route
void PathUpdater::ClearPrevPath() {
  prev_path_.clear();
}

/// Search for the starting position from the previous route
/// If not found, output the current position as the starting position
std::optional<uint32_t> PathUpdater::SearchStartPoseOnPrevPath(const Pose2d& global_pose, Pose2d& start_pose) {
  std::optional<uint32_t> start_index_on_prev_path = SearchNearestPointIndexOnPath(prev_path_, global_pose);
  if (!start_index_on_prev_path) {
    /// If the nearest point is not found, set the current position as the starting position
    start_pose = global_pose;
    return std::nullopt;
  }

  const double distance = (global_pose.point() - prev_path_[start_index_on_prev_path.value()].point()).norm();
  if (distance - param_.distance_on_prev_path < std::numeric_limits<double>::epsilon()) {
    /// If the nearest point on the previous route is within range, set it as the starting position
    start_pose = prev_path_[start_index_on_prev_path.value()];
    return start_index_on_prev_path;
  } else {
    /// If the nearest point on the previous route is out of range, set the current position as the starting position
    start_pose = global_pose;
    return std::nullopt;
  }
}

/// Update the route based on the previous route and the input route
bool PathUpdater::UpdatePath(const PoseSeq& path, const std::optional<uint32_t>& start_index_on_prev_path,
                             PoseSeq& update_path) {
  if (!start_index_on_prev_path) {
    // If the nearest point is not found, output the input route and return that an update is needed
    update_path = path;
    prev_path_ = update_path;
    return true;
  }

  // Check if an update is necessary
  const PathUpdater::CheckUpdatePathResult check_update = CheckUpdatePath_(path, start_index_on_prev_path.value());
  if (check_update == kNoUpdate) {
    // If no update is needed, output the previous route and return that no update is needed
    update_path = prev_path_;
    return false;
  } else if (check_update == kMergeUpdate) {
    // In case of partial update, concatenate the previous route and the input route, output it, and return that an update is needed
    update_path.resize(start_index_on_prev_path.value() + path.size());
    for (uint32_t i = 0; i < start_index_on_prev_path.value(); i++) {
      update_path[i] = prev_path_[i];
    }
    for (uint32_t i = 0; i < path.size(); i++) {
      update_path[i + start_index_on_prev_path.value()] = path[i];
    }
  } else {
    // In case of full update, output the input route and return that an update is needed
    update_path = path;
  }
  prev_path_ = update_path;
  return true;
}

// Compare the previous route and the input route to check if an update is necessary
PathUpdater::CheckUpdatePathResult PathUpdater::CheckUpdatePath_(
    const PoseSeq& path, const uint32_t start_index_on_prev_path) {
  if ((path.back().point() - prev_path_.back().point()).norm() > std::numeric_limits<double>::epsilon() ||
      fabs(path.back().theta() - prev_path_.back().theta()) > std::numeric_limits<double>::epsilon()) {
    // Perform a full update if the goal position and orientation do not match
    return kAllUpdate;
  }

  // Check if the current route matches the previous route from the back
  for (uint32_t i = 0; i < path.size() - 1 && i < prev_path_.size() - start_index_on_prev_path - 1; i++) {
    const Pose2d prev_path_pose = prev_path_[start_index_on_prev_path + i];
    const Pose2d path_pose = path[i];
    if (fabs(prev_path_pose.x() - path_pose.x()) - param_.grid_error > std::numeric_limits<double>::epsilon() ||
        fabs(prev_path_pose.y() - path_pose.y()) - param_.grid_error > std::numeric_limits<double>::epsilon()) {
      // Determine whether to perform a full update or partial update based on how many points match
      if (i > static_cast<uint32_t>(param_.same_point_num_merge_path)) {
        // Perform a partial update if some parts of the route match
        return kMergeUpdate;
      } else {
        // Perform a full update if most of the route is different
        return kAllUpdate;
      }
    }
  }
  // No update is needed if the routes match from the back
  return kNoUpdate;
}
}  // namespace tmc_base_path_planner
