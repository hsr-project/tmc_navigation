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
#ifndef TMC_BASE_PATH_FOLLOWER_NEAREST_PATH_POINT_SEARCHER_HPP_
#define TMC_BASE_PATH_FOLLOWER_NEAREST_PATH_POINT_SEARCHER_HPP_
#include <memory>
#include <optional>
#include <vector>

#include <console_bridge/console.h>
#include "common.hpp"
#include "parameter_default_value.hpp"


namespace tmc_base_path_follower {

class INearestPathPointSearcher {
 public:
  using Ptr = std::shared_ptr<INearestPathPointSearcher>;
  virtual ~INearestPathPointSearcher() = default;
  virtual uint32_t SearchNearestPathPointIndex(
      const PoseSeq& path, const std::vector<double>& left_path_lengths,
      const Pose2d& global_pose, const std::optional<uint32_t>& prev_index) = 0;
};

/// Nearest Route Point Search Class
class NearestPathPointSearcher : public INearestPathPointSearcher {
 public:
  using Ptr = std::shared_ptr<NearestPathPointSearcher>;
  /// Parameters
  struct Parameter {
    Parameter(const double in_partial_search_range,
              const double in_partial_search_permit_error)
        : partial_search_range(in_partial_search_range),
          partial_search_permit_error(in_partial_search_permit_error) {
      if (partial_search_range < 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'partial_search_range' is invalid. Use default value.");
        partial_search_range = kPartialSearchRangeDefault;
      }
      if (partial_search_permit_error <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'partial_search_permit_error' is invalid. Use default value.");
        partial_search_permit_error = kPartialSearchPermitErrorDefault;
      }
    }
    // Partial search range from the previous nearest point [m]
    double partial_search_range;
    // Allowable error value between the partially searched route point and self-position [m]
    double partial_search_permit_error;
  };

  /// Constructor
  /// @param [I] param Parameters
  explicit NearestPathPointSearcher(const Parameter& param) : param_(param) {}

  /// Nearest Route Point Search
  /// @param [I] path Route
  /// @param [I] left_path_lengths Remaining distance to the goal for each route point
  /// @param [I] global_pose Self-position
  /// @param [I] prev_index Previous nearest point information
  /// @return Nearest point index
  uint32_t SearchNearestPathPointIndex(const PoseSeq& path, const std::vector<double>& left_path_lengths,
                                       const Pose2d& global_pose, const std::optional<uint32_t>& prev_index);

 private:
  /// Range Search for Nearest Route Point
  /// Search for the nearest point within the specified index range
  /// @param [I] path Route
  /// @param [I] global_pose Self-position
  /// @param [I] search_first_index Search start index
  /// @param [I] search_last_index Search end index
  /// @ret Nearest point index
  uint32_t SearchNearestPathPointIndexInRange(
      const PoseSeq& path, const Pose2d& global_pose,
      const uint32_t search_first_index, const uint32_t search_last_index);

  // Parameters
  Parameter param_;
};

}  // namespace tmc_base_path_follower

#endif  // TMC_BASE_PATH_FOLLOWER_NEAREST_PATH_POINT_SEARCHER_HPP_
