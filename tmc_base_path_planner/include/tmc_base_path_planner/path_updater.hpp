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
#ifndef TMC_BASE_PATH_PLANNER_PATH_UPDATER_HPP_
#define TMC_BASE_PATH_PLANNER_PATH_UPDATER_HPP_
#include <memory>
#include <optional>

#include <console_bridge/console.h>

#include "common.hpp"

namespace tmc_base_path_planner {
class IPathUpdater {
 public:
  using Ptr = std::shared_ptr<IPathUpdater>;
  virtual ~IPathUpdater() = default;
  virtual std::optional<uint32_t> SearchStartPoseOnPrevPath(const Pose2d& global_pose, Pose2d& start_pose) = 0;
  virtual void ClearPrevPath() = 0;
  virtual bool UpdatePath(const PoseSeq& path, const std::optional<uint32_t>& start_index_on_prev_path,
                          PoseSeq& update_path) = 0;
};

/// Path Update Class
class PathUpdater : public IPathUpdater {
 public:
  /// PathUpdater Parameters
  struct Parameter {
    Parameter(const double in_distance_on_prev_path, const double in_grid_error,
              const int32_t in_same_point_num_merge_path)
        : distance_on_prev_path(in_distance_on_prev_path), grid_error(in_grid_error),
          same_point_num_merge_path(in_same_point_num_merge_path) {
      if (distance_on_prev_path <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of '%s' is invalid. Use default value.", kDistanceOnPrevPathName);
        distance_on_prev_path = kDistanceOnPrevPathDefault;
      }
      if (grid_error < 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of '%s' is invalid. Use default value.", kGridErrorName);
        grid_error = kGridErrorDefault;
      }
      if (same_point_num_merge_path < 0) {
        CONSOLE_BRIDGE_logWarn("Value of '%s' is invalid. Use default value.", kSamePointNumMergePathName);
        same_point_num_merge_path = kSamePointNumMergePathDefault;
      }
    }
    // Distance to the previous path to be considered on the previous path [m]
    double distance_on_prev_path;
    // Allowable deviation between each path point when determining path update [m]
    double grid_error;
    // Threshold for merging paths if they match up to a certain number of points [num]
    int32_t same_point_num_merge_path;
  };

  /// Constructor
  /// @param [I] param Parameters
  explicit PathUpdater(const Parameter& param) : param_(param) {}

  /// Clear the previous path
  void ClearPrevPath();

  /// Find the start position from the previous path
  /// If not found, output the self-position as the start position
  /// @param [I] global_pose Self-position
  /// @param [O] start_pose Start position
  /// @return Index on the previous path of the start position
  std::optional<uint32_t> SearchStartPoseOnPrevPath(const Pose2d& global_pose, Pose2d& start_pose);

  /// Update the path based on the previous path and the input path
  /// @param [I] path Input path
  /// @param [I] start_index_on_prev_path Index on the previous path of the start position
  /// @param [O] update_path Updated path
  /// @return true: Update needed false: No update needed
  bool UpdatePath(const PoseSeq& path, const std::optional<uint32_t>& start_index_on_prev_path,
                  PoseSeq& update_path);

 private:
  /// Result of CheckUpdatePath_
  enum CheckUpdatePathResult {
    /// No update needed
    kNoUpdate,
    /// Partial update
    kMergeUpdate,
    /// Full update
    kAllUpdate
  };
  /// Compare the previous path and the input path to determine if an update is needed
  /// @param[I] path Input path
  /// @param[I] start_index_on_prev_path Index on the previous path of the start position of the input path
  /// @return kNoUpdate: No update needed kMergeUpdate: Partial update kAllUpdate: Full update
  CheckUpdatePathResult CheckUpdatePath_(const PoseSeq& path, const uint32_t start_index_on_prev_path);

  // Previous path
  PoseSeq prev_path_;
  // Parameters
  Parameter param_;
};

}  // namespace tmc_base_path_planner

#endif  // TMC_BASE_PATH_PLANNER_PATH_UPDATER_HPP_
