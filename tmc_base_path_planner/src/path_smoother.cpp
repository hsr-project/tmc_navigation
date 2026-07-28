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
#include <tmc_base_path_planner/path_smoother.hpp>

#include <limits>

namespace {
// FIR filter order for smoothing process, must be a multiple of 2
constexpr int32_t kFilterOrder = 20;
}  // anonymous namespace

namespace tmc_base_path_planner {

/// Outputs a smoothed input path with assigned direction
bool PathSmoother::SmoothingPath(const PoseSeq& input_path, PoseSeq& output_path) {
  // No interpolation, only smoothing (number of input and output path points remains the same)
  output_path.resize(input_path.size());
  // Goal is not updated
  output_path[output_path.size() - 1] = input_path.back();

  // Smoothing process
  for (size_t i = 0; i < output_path.size() - 1; i++) {
    FilterPathPoint_(input_path, static_cast<double>(i), output_path[i]);
  }

  // Calculate path direction
  for (uint32_t i = 0; i < output_path.size() - 1; i++) {
    double path_angle;
    if (i > 0) {
      path_angle = atan2(output_path[i].y() - output_path[i - 1].y(),
                         output_path[i].x() - output_path[i - 1].x());
    } else {
      path_angle = atan2(output_path[i + 1].y() - output_path[i].y(),
                         output_path[i + 1].x() - output_path[i].x());
    }
    output_path[i].set_theta(path_angle);
  }
  return true;
}

/// Calculate the coordinates of a single point at an arbitrary location on the smoothed & interpolated input path
void PathSmoother::FilterPathPoint_(const PoseSeq& input_path, const double index_to_filter, Pose2d& filtered_pose) {
  // Integer part of the interpolation target position
  const int32_t index_int = static_cast<int32_t>(std::floor(index_to_filter));
  // Fractional part of the interpolation target position
  const double index_frac = index_to_filter - index_int;

  if (input_path.size() == 1) {
    // Cannot interpolate if there is only one input data point
    filtered_pose = input_path.front();
  } else if (index_to_filter < std::numeric_limits<double>::epsilon()) {
    // Do not filter the first element
    filtered_pose = input_path.front();
  } else if (index_int == static_cast<int32_t>(input_path.size() - 1)) {
    // Do not filter the last element
    filtered_pose = input_path.back();
  } else {
    const int32_t filter_order_2 = kFilterOrder / 2;

    // Generate filter coefficients
    double taps[kFilterOrder];
    for (int32_t i = 0; i < filter_order_2; ++i) {
      const double dtt = (index_frac + static_cast<double>(i)) / static_cast<double>(filter_order_2);
      taps[kFilterOrder - 1 - i] = dtt * dtt * dtt * (dtt * (dtt * 6.0 - 15.0) + 10.0);
      taps[filter_order_2 - 1 - i] = 1.0 - taps[kFilterOrder - 1 - i];
    }

    // If the filter range exceeds the data portion, set the exceeded part to 0 and shift the weight inward
    if (index_int < (filter_order_2 - 1)) {
      // When exceeding forward
      const int32_t under = filter_order_2 - 1 - index_int;
      for (int32_t i = 0; i < under; ++i) {
        const double dtt = index_to_filter / static_cast<double>(i + 1 + index_int);
        taps[i] = 0.0;
        taps[filter_order_2 + i] = (dtt * dtt * dtt * (dtt * (dtt * 6.0 - 15.0) + 10.0));
      }
    } else if (index_int >= static_cast<int32_t>(input_path.size() - filter_order_2)) {
      // When exceeding backward
      const int32_t over = (index_int + filter_order_2) - (input_path.size() - 1);
      for (int32_t i = 0; i < over; i++) {
        const double dtt = (index_frac + static_cast<double>(i)) / static_cast<double>(filter_order_2 - over + i);
        taps[kFilterOrder - 1 - i] = 0.0;
        taps[filter_order_2 - 1 - i] = 1.0 - (dtt * dtt * dtt * (dtt * (dtt * 6.0 - 15.0) + 10.0));
      }
    }

    // Convolve input data with filter coefficients and take the weighted average
    double accum_x = 0.0;
    double accum_y = 0.0;
    double sum = 0.0;
    for (int32_t i = 0; i < kFilterOrder; ++i) {
      const int32_t data_index = index_int - filter_order_2 + 1 + i;
      if ((data_index >= 0) && (data_index < static_cast<int32_t>(input_path.size()))) {
        sum += taps[i];
        accum_x += taps[i] * input_path[data_index].x();
        accum_y += taps[i] * input_path[data_index].y();
      }
    }
    filtered_pose.set_x(accum_x / sum);
    filtered_pose.set_y(accum_y / sum);
  }
}

}  // namespace tmc_base_path_planner
