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
#ifndef TMC_BASE_PATH_PLANNER_PATH_SMOOTHER_HPP_
#define TMC_BASE_PATH_PLANNER_PATH_SMOOTHER_HPP_
#include <memory>

#include "common.hpp"

namespace tmc_base_path_planner {

class IPathSmoother {
 public:
  using Ptr = std::shared_ptr<IPathSmoother>;
  virtual ~IPathSmoother() = default;
  virtual bool SmoothingPath(const PoseSeq& input_path, PoseSeq& output_path) = 0;
};

/// Path smoothing class
class PathSmoother : public IPathSmoother {
 public:
  /// Constructor
  PathSmoother() = default;

  /// Outputs a smoothed input path with assigned path orientation
  /// @param [I] input_path Input path
  /// @param [O] output_path Output path
  /// @return Success/Failure
  bool SmoothingPath(const PoseSeq& input_path, PoseSeq& output_path);

 private:
  /// Calculates the coordinates of a single point at any location from the smoothed & interpolated input path
  /// @param [I] input_path Input path
  /// @param [I] index_to_filter Index position to calculate (e.g., if you want the midpoint between the 4th and 5th points, provide 4.5)
  /// @param [O] filtered_pose Calculated coordinates with x, y only; orientation is not processed
  /// @return None
  void FilterPathPoint_(const PoseSeq& input_path, const double index_to_filter, Pose2d& filtered_pose);
};
}  // namespace tmc_base_path_planner
#endif  // TMC_BASE_PATH_PLANNER_PATH_SMOOTHER_HPP_
