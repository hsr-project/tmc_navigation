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
#include <tmc_base_path_follower/nearest_path_point_searcher.hpp>

#include <limits>
#include <vector>
namespace tmc_base_path_follower {

/// Route Nearest Point Search
uint32_t NearestPathPointSearcher::SearchNearestPathPointIndex(
    const PoseSeq& path, const std::vector<double>& left_path_lengths,
    const Pose2d& global_pose, const std::optional<uint32_t>& prev_index) {
  double error = std::numeric_limits<double>::max();
  uint32_t nearest_path_point_index = 0;
  // If partial search is enabled and previous nearest point information is available, search around that area
  if (param_.partial_search_range > std::numeric_limits<double>::epsilon() && prev_index) {
    // Determine the search range for partial search
    uint32_t search_first_index = prev_index.value();
    // Search start position
    while (search_first_index > 0) {
      if (fabs(left_path_lengths[search_first_index] - left_path_lengths[prev_index.value()]) >
          param_.partial_search_range) {
        break;
      }
      --search_first_index;
    }
    // Search end position
    uint32_t search_last_index = prev_index.value();
    while (search_last_index < path.size() - 1) {
      if (fabs(left_path_lengths[search_last_index] - left_path_lengths[prev_index.value()]) >
          param_.partial_search_range) {
        break;
      }
      ++search_last_index;
    }
    // Search for the nearest point within the range
    nearest_path_point_index = SearchNearestPathPointIndexInRange(path, global_pose,
        search_first_index, search_last_index);
    // Calculate the position difference with the found point
    error = (path[nearest_path_point_index].point() - global_pose.point()).norm();
  }
  // If not performing partial search, or if the nearest point found in partial search is not closer than the threshold, perform full search
  if (error > param_.partial_search_permit_error) {
    nearest_path_point_index = SearchNearestPathPointIndexInRange(path, global_pose, 0, path.size() - 1);
  }
  return nearest_path_point_index;
}

/// Range Search for Route Nearest Point
/// Search for the nearest point within the specified index range
uint32_t NearestPathPointSearcher::SearchNearestPathPointIndexInRange(
    const PoseSeq& path, const Pose2d& global_pose,
    const uint32_t search_first_index, const uint32_t search_last_index) {
  uint32_t nearest_index = search_first_index;
  double minimum_error = std::numeric_limits<double>::max();
  // Calculate the difference between each route point and the robot position
  for (uint32_t i = search_first_index; i <= search_last_index; ++i) {
    // Store the magnitude of the position deviation
    const double difference = (path[i].point() - global_pose.point()).norm();
    if (difference < minimum_error) {
      nearest_index = i;
      minimum_error = difference;
    }
  }
  return nearest_index;
}
}  // namespace tmc_base_path_follower
