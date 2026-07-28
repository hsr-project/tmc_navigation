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
#ifndef TMC_MAP_MERGER_MAP_PROJECTOR_HPP_
#define TMC_MAP_MERGER_MAP_PROJECTOR_HPP_

#include <cmath>
#include <stdint.h>
#include <algorithm>
#include <vector>

#include "map_merger.hpp"

namespace tmc_map_merger {

namespace { // NOLINT
const double kEpsilon = 0.001;
}  // anonymous namespace

/// @brief Position
template<typename T = double>
struct Pos {
  T x;
  T y;
};

/// @brief Size
template<typename T = double>
struct Size {
  T width;
  T height;
};

/// @brief Rectangle
template<typename T = double>
struct Rect {
  Pos<T> min;
  Pos<T> max;
};

/// @brief WeightStrategy that directly passes the input-side cost
struct WeightFixed {
  int8_t Weight(const int8_t value, const Size<>& src, const Size<>& dst) {
    return value;
  }
};

/// @breif Class for calculating the position and overlap of the input map relative to the output map
// To simplify calculations, normalize the size of the output grid to 1
// Shift the input position so that the origin of the output grid is at (0, 0) and theta=0
template<typename SrcMapOp, typename DstMapOp>
struct GridMap {
  // Constructor
  GridMap(const SrcMapOp& srcOp, const DstMapOp& dstOp) {
    // Normalize so that the size of the output grid becomes 1
    res = srcOp.GetInfo().resolution / dstOp.GetInfo().resolution;
    // Angle of the input map relative to the output map
    double src_theta = GetYawFromQuaternion(srcOp.GetInfo().origin.orientation);
    double dst_theta = GetYawFromQuaternion(dstOp.GetInfo().origin.orientation);
    double theta = src_theta - dst_theta;
    cos = res * std::cos(theta);
    sin = res * std::sin(theta);
    // Origin of the input map relative to the output map
    double x0 = (srcOp.GetInfo().origin.position.x - dstOp.GetInfo().origin.position.x) / dstOp.GetInfo().resolution;
    double y0 = (srcOp.GetInfo().origin.position.y - dstOp.GetInfo().origin.position.y) / dstOp.GetInfo().resolution;
    origin.x =  x0 * std::cos(dst_theta) + y0 * std::sin(dst_theta);
    origin.y = -x0 * std::sin(dst_theta) + y0 * std::cos(dst_theta);
    // Size
    dst_size.width = dstOp.GetInfo().width;
    dst_size.height = dstOp.GetInfo().height;
    // Boundary position of one grid of the input map (used to limit the overlapping range of the output)
    // Maintain the minimum and maximum of the following four coordinates
    // (0, 0), (cos, sin), (cos - sin, cos + sin), (-sin, cos)
    area.min.x = std::min(std::min(0.0, cos), std::min(cos - sin, -sin));
    area.max.x = std::max(std::max(0.0, cos), std::max(cos - sin, -sin));
    area.min.y = std::min(std::min(0.0, sin), std::min(cos + sin, cos));
    area.max.y = std::max(std::max(0.0, sin), std::max(cos + sin, cos));
  }

  // @brief Given the grid position (x, y) of the input, return the range of overlapping grid indices on the output side
  // Saturate the output from 0 to the maximum value
  // (If out of range, min=max=0 or min=max=maximum value)
  Rect<uint32_t> GetOverlappedRange(uint32_t src_x, uint32_t src_y) {
    Rect<uint32_t> range;
    // Determine the origin of the input grid
    double x = origin.x + src_x * cos - src_y * sin;
    double y = origin.y + src_x * sin + src_y * cos;
    // Determine the overlapping output grid from the origin
    range.min.x = std::min(static_cast<size_t>(std::max(0, static_cast<int>(x + area.min.x))), dst_size.width);
    range.max.x = std::min(static_cast<size_t>(std::max(0, static_cast<int>(x + area.max.x) + 1)), dst_size.width);
    range.min.y = std::min(static_cast<size_t>(std::max(0, static_cast<int>(y + area.min.y))), dst_size.height);
    range.max.y = std::min(static_cast<size_t>(std::max(0, static_cast<int>(y + area.max.y) + 1)), dst_size.height);
    return range;
  }

  /// @brief Obtain the width and height of the overlap when projecting the input grid onto the output grid
  Size<> GetOverlappedGridSize(uint32_t src_x, uint32_t src_y, uint32_t dst_x, uint32_t dst_y) {
    Size<> size;
    // Determine the origin of the input grid
    double x = origin.x + src_x * cos - src_y * sin;
    double y = origin.y + src_x * sin + src_y * cos;
    // First, obtain the overlapping length in the x-direction
    // The input grid range is (x-min, x+max), and the output grid range is (dst_x, dst_x+1)
    // The overlapping range is (max(x-min, dst_x), min(x+max, dst_x+1))
    // If negative, there is no overlap, so set it to 0.0
    size.width = std::max(0.0,
                          std::min(x + area.max.x, static_cast<double>(dst_x + 1)) -
                          std::max(x + area.min.x, static_cast<double>(dst_x)));
    // The same applies to the y-direction
    size.height = std::max(0.0,
                          std::min(y + area.max.y, static_cast<double>(dst_y + 1)) -
                          std::max(y + area.min.y, static_cast<double>(dst_y)));
    return size;
  }
  double res;
  double cos;
  double sin;
  Pos<> origin;
  Rect<> area;
  Size<size_t> dst_size;
};

/// @brief Perform parallel translation of the map in grid units
/// No pre-checks are performed
template<typename SrcMapOp, typename DstMapOp>
void ProjectGrid(const SrcMapOp& srcOp, const DstMapOp& dstOp) {
  int32_t diff_x = (dstOp.GetInfo().origin.position.x -
                    srcOp.GetInfo().origin.position.x) / srcOp.GetInfo().resolution;
  int32_t diff_y = (dstOp.GetInfo().origin.position.y -
                    srcOp.GetInfo().origin.position.y) / srcOp.GetInfo().resolution;
  Pos<int32_t> src_pos;
  Pos<int32_t> dst_pos;
  size_t dst_index;
  // Change the copy direction to allow copying of the same data
  if (diff_y < 0 || (diff_y == 0 && diff_x < 0)) {
    dst_index = dstOp.GetInfo().width * dstOp.GetInfo().height - 1;
    for (dst_pos.y = dstOp.GetInfo().height - 1, src_pos.y = dst_pos.y + diff_y;
         dst_pos.y >= 0;
         --dst_pos.y, --src_pos.y) {
      for (dst_pos.x = dstOp.GetInfo().width - 1, src_pos.x = dst_pos.x + diff_x;
           dst_pos.x >= 0;
           --dst_pos.x, --src_pos.x) {
        if (src_pos.x >= 0 && src_pos.x < srcOp.GetInfo().width &&
            src_pos.y >= 0 && src_pos.y < srcOp.GetInfo().height) {
          size_t src_index = src_pos.y * srcOp.GetInfo().width + src_pos.x;
          dstOp.Update(dst_index, srcOp.Get(src_index));
        } else {
          dstOp.Reset(dst_index);
        }
        --dst_index;
      }
    }
  } else {
    dst_index = 0;
    for (dst_pos.y = 0, src_pos.y = dst_pos.y + diff_y;
         dst_pos.y < dstOp.GetInfo().height;
         ++dst_pos.y, ++src_pos.y) {
      for (dst_pos.x = 0, src_pos.x = dst_pos.x + diff_x;
           dst_pos.x < dstOp.GetInfo().width;
           ++dst_pos.x, ++src_pos.x) {
        if (src_pos.x >= 0 && src_pos.x < srcOp.GetInfo().width &&
            src_pos.y >= 0 && src_pos.y < srcOp.GetInfo().height) {
          size_t src_index = src_pos.y * srcOp.GetInfo().width + src_pos.x;
          dstOp.Update(dst_index, srcOp.Get(src_index));
        } else {
          dstOp.Reset(dst_index);
        }
        ++dst_index;
      }
    }
  }
}


struct Copy {
  int8_t Get(int8_t data, double src_overlapped_area, double dst_overlad_area) const {
    return data;
  }
};

/// @brief Perform projection of the map
/// SrcMapOp and DstMapOp are classes that implement memory
/// For optimization (to avoid function pointer calls)
/// Implement polymorphism using templates
/// Implement the following methods
///  int8_t Get(const ros::Time& time) const         : Retrieve the current memory
///  void Update(int8_t data, const ros::Time& time) : Update the memory
///  void Reset(const ros::Time& time)               : Reset the memory

template<typename SrcMapOp, typename DstMapOp, typename WeightStrategy>
void Project(const SrcMapOp& srcOp, const DstMapOp& dstOp,
             const WeightStrategy& weight_strategy) {
  // resolution matches
  if (srcOp.GetInfo().resolution == dstOp.GetInfo().resolution) {
    // and x, y offsets are integer multiples of the resolution
    double ix = std::fabs(srcOp.GetInfo().origin.position.x - dstOp.GetInfo().origin.position.x) /
        srcOp.GetInfo().resolution;
    double iy = std::fabs(srcOp.GetInfo().origin.position.y - dstOp.GetInfo().origin.position.y) /
        srcOp.GetInfo().resolution;
    if ((std::fabs(std::floor(ix + 0.5) - ix) < kEpsilon) &&
        (std::fabs(std::floor(iy + 0.5) - iy) < kEpsilon)) {
      double src_theta = GetYawFromQuaternion(srcOp.GetInfo().origin.orientation);
      double dst_theta = GetYawFromQuaternion(dstOp.GetInfo().origin.orientation);
      // and the direction is the same
      if (std::fabs(src_theta - dst_theta) < kEpsilon) {
        // Perform projection in grid units
        ProjectGrid(srcOp, dstOp);
        return;
      }
    }
  }
  GridMap<SrcMapOp, DstMapOp> src_grid(srcOp, dstOp);
  GridMap<DstMapOp, SrcMapOp> dst_grid(dstOp, srcOp);
  // Loop on the output side (scan all)
  size_t dst_index = 0;
  Pos<size_t> dst_pos;
  for (dst_pos.y = 0;
       dst_pos.y < dstOp.GetInfo().height;
       ++dst_pos.y) {
    for (dst_pos.x = 0;
         dst_pos.x < dstOp.GetInfo().width;
         ++dst_pos.x) {
      // Get the range of overlapping indices in the input grid
      Rect<uint32_t> range = dst_grid.GetOverlappedRange(dst_pos.x, dst_pos.y);
      Pos<size_t> src_pos;
      if (range.min.x != range.max.x && range.min.y != range.max.y) {
        for (src_pos.y = range.min.y;
             src_pos.y < range.max.y;
             ++src_pos.y) {
          for (src_pos.x = range.min.x;
               src_pos.x < range.max.x;
               ++src_pos.x) {
            // Check if grids overlap
            // Project onto each other's edge directions and determine overlap if all overlap
            // Algorithm is as follows
            // https://www.codeproject.com/Articles/15573/2D-Polygon-Collision-Detection

            // Overlap of the output side relative to the input
            Size<> dst_overlapped = dst_grid.GetOverlappedGridSize(dst_pos.x, dst_pos.y, src_pos.x, src_pos.y);
            // Overlap of the input side relative to the output
            Size<> src_overlapped = src_grid.GetOverlappedGridSize(src_pos.x, src_pos.y, dst_pos.x, dst_pos.y);
            if (src_overlapped.width > 0.0 &&
                src_overlapped.height > 0.0 &&
                dst_overlapped.width > 0.0 &&
                dst_overlapped.height > 0.0) {
              // Overlapping
              size_t src_index  = src_pos.y * srcOp.GetInfo().width + src_pos.x;
              int8_t new_data = weight_strategy.Get(srcOp.Get(src_index),
                                                    src_overlapped.width * src_overlapped.height,
                                                    dst_overlapped.width * dst_overlapped.height);
              dstOp.Update(dst_index, new_data);
            }
          }
        }
      } else {
        dstOp.Reset(dst_index);
      }
      ++dst_index;
    }
  }
}

template<typename SrcMapOp, typename DstMapOp>
void Project(const SrcMapOp& srcOp, const DstMapOp& dstOp) {
  Project(srcOp, dstOp, Copy());
}

}  // namespace tmc_map_merger

#endif
