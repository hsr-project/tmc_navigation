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
#ifndef TMC_POSE_2D_LIB_DISTANCE_MAP_HPP_
#define TMC_POSE_2D_LIB_DISTANCE_MAP_HPP_

#include <math.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "tmc_pose_2d_lib/pose_2d.hpp"

namespace tmc_pose_2d_lib {

/// Distance map data
class DistanceMap {
 public:
  // Constructor
  DistanceMap(
    Pose2d origin_map_image,
    double resolution,
    size_t width,
    size_t height,
    std::vector<unsigned char> data);


  enum CellType {
    kOutside = 0,
    kUnExplored,
    kFree,
    kHasDistance,
  };

  /// getter
  double resolution() const {return resolution_;}
  size_t width() const {return width_;}
  size_t height() const {return height_;}
  size_t min_u() const {return min_u_;}
  size_t min_v() const {return min_v_;}
  size_t max_u() const {return max_u_;}
  size_t max_v() const {return max_v_;}
  Pose2d origin() const { return origin_map_image_; }
  std::vector<unsigned char> data() const { return data_; }

  /// setter
  void SetPotentialWidth(double potential_width) {
    potential_width_ = potential_width;
  }
  /// @brief Set map data value
  bool SetValueAt(const size_t index_u, const size_t index_v, const unsigned char value);
  /// @brief Get map data value
  bool GetValueAt(const size_t index_u, const size_t index_v, unsigned char& value) const;
  /// @brief Obtain coordinate values in map coordinate system from values in image coordinate system
  bool GetMapPoint(const size_t index_u, const size_t index_v, Point2d& point_map) const;
  /// @brief Determine whether the specified 2D point in map coordinate system is within map data range
  bool InMap(const Point2d& p_map) const;
  /// @brief Check the Type of image data at the specified map coordinate point and return the value if a valid distance is entered
  void CheckTypeAndDistance(const Point2d& p_map, DistanceMap::CellType& type, double& distance) const;
  /// @brief Check area with obstacles
  void CalcOccupiedRegion();
  /// @brief Coordinate transformation from image coordinate system to map coordinate system
  void ImageToMap(Pose2d& pose);
  /// @brief Coordinate transformation from map coordinate system to image coordinate system
  void MapToImage(Pose2d& pose);
  /// @brief Expand obstacle area such that occupancy decreases as it moves away from the wall to a specified distance
  void InflateMap(const double potential_width);

 private:
  /// map->image coordinate system Transformation from image coordinate system to map coordinate system
  Pose2d origin_map_image_;
  /// map->image coordinate system Inverse transformation from image coordinate system to map coordinate system
  Pose2d origin_map_image_inverse_;
  /// Grid size (m)
  double resolution_;
  /// Width of the map (number of grids)
  size_t width_;
  /// Height of the map (number of grids)
  size_t height_;

  /// @brief Map data
  std::vector<unsigned char> data_;

  /// Occupied and non-occupied threshold
  // Normalizing (255-data) and multiplying by potential_width gives the shortest distance to an object in that cell
  double potential_width_;

  // Range of indices where potential values exist
  size_t min_u_;
  size_t min_v_;
  size_t max_u_;
  size_t max_v_;

  /// @brief Obtain image coordinate system value for specified 2D point in map coordinate system
  bool CheckIndices(const Point2d& p_map, size_t& index_u, size_t& index_v) const;
  bool CheckIndexArea(const size_t index_u, const size_t index_v) const;

  bool IsValid() const;

  /// Calculate grid distance to the nearest wall for each grid
  std::vector<double> CalculateGridDistanceToWalls();
  /// Update of distance to nearest wall and nearest wall location information
  void UpdateDistanceAndNearestWall(const int32_t x, const int32_t y, const int32_t neighbor_index,
                                    std::vector<int32_t>& nearest_wall_indexes, double& distance);
};

}  // namespace tmc_pose_2d_lib

#endif  // TMC_POSE_2D_LIB_DISTANCE_MAP_HPP_
