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
#include "tmc_pose_2d_lib/distance_map.hpp"
#include <climits>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

namespace {
const int32_t kInvalidNearestWall = std::numeric_limits<int32_t>::max();

const unsigned char kWallValue = 255;
const unsigned char kFreeValue = 1;
const unsigned char kUnexploredValue = 0;

/// Function to calculate Euclidean distance
double CalculateDistance(const int32_t x1, const int32_t y1, const int32_t x2, const int32_t y2) {
  return sqrt(static_cast<double>(
              (static_cast<double>(x1) - static_cast<double>(x2)) *
              (static_cast<double>(x1) - static_cast<double>(x2)) +
              (static_cast<double>(y1) - static_cast<double>(y2)) *
              (static_cast<double>(y1) - static_cast<double>(y2))));
}
}  // anonymous namespace

namespace tmc_pose_2d_lib {
/// @brief Constructor
DistanceMap::DistanceMap(
  const Pose2d origin_map_image,
  const double resolution,
  const size_t width,
  const size_t height,
  const std::vector<unsigned char> data)
  : origin_map_image_(origin_map_image),
    resolution_(resolution),
    width_(width),
    height_(height),
    data_(data) {
  CalcOccupiedRegion();
  // Check if the number of map data arrays is correct
  if (!IsValid()) {
    std::cerr << "Map size is not correct. width=" << width_ << "height=" << height_
              << "size=" << data_.size() << std::endl;
    throw std::runtime_error("Map info is invalid!");
  }
  // Pre-calculate to reduce computation time
  origin_map_image_inverse_ = origin_map_image_.Inverse();
}

bool DistanceMap::IsValid() const {
  return width_ * height_ == data_.size();
}

bool DistanceMap::CheckIndexArea(const size_t index_u, const size_t index_v) const {
  return (index_u < width_) && (index_u >= 0) && (index_v < height_) && (index_v >= 0);
}

bool DistanceMap::SetValueAt(const size_t index_u, const size_t index_v, const unsigned char value) {
  if (!CheckIndexArea(index_u, index_v)) {
    return false;
  }
  data_[index_u + index_v * width_] = value;
  return true;
}

bool DistanceMap::GetValueAt(const size_t index_u, const size_t index_v, unsigned char& value) const {
  if (!CheckIndexArea(index_u, index_v)) {
    return false;
  }
  value = data_[index_u + index_v * width_];
  return true;
}

bool DistanceMap::GetMapPoint(const size_t index_u, const size_t index_v, Point2d& point_map) const {
  if (!CheckIndexArea(index_u, index_v)) {
    return false;
  }
  const Point2d point_image = Point2d(index_u * resolution_, index_v * resolution_);
  point_map = origin_map_image_ * point_image;
  return true;
}

bool DistanceMap::CheckIndices(const Point2d& p_map, size_t& index_u, size_t& index_v) const {
  const Point2d p_image = origin_map_image_inverse_ * p_map;
  // int(1.3) = 1 leads to truncation (rounds down), but
  // In the case of negative numbers, int(-1.3) = -1 rounds up, so casting negative numbers should be done carefully
  if (p_image.x() < 0.0 || p_image.y() < 0.0) {
    return false;
  }

  index_u = static_cast<size_t>(p_image.x() / resolution_);
  index_v = static_cast<size_t>(p_image.y() / resolution_);

  // Confirm that it is within the image index
  return CheckIndexArea(index_u, index_v);
}

bool DistanceMap::InMap(const Point2d& p_map) const {
  size_t index_u, index_v;
  return CheckIndices(p_map, index_u, index_v);
}

void DistanceMap::CheckTypeAndDistance(const Point2d& p_map, DistanceMap::CellType& type, double& distance) const {
  distance = 0.0;
  size_t index_u, index_v;

  if (CheckIndices(p_map, index_u, index_v)) {
    const unsigned char val = data_[index_u + index_v * width_];
    if (val == kFreeValue) {
      type = kFree;
      return;
    } else if (val == kUnexploredValue) {
      type = kUnExplored;
      return;
    }
    // Normalizing (255-val) and multiplying by potential_width_ turns into distance
    // UCHAR_MAX = 255
    distance = static_cast<double>((UCHAR_MAX - val)) / static_cast<double>(UCHAR_MAX) * potential_width_;
    type = kHasDistance;
    return;
  } else {
    type = kOutside;
    return;
  }
}

/// Expand the obstacle area so that occupancy rate decreases as it moves away from the wall to the specified distance
void DistanceMap::InflateMap(const double potential_width) {
  const double potential_grid_width = potential_width / resolution_;
  if (potential_grid_width < std::numeric_limits<double>::epsilon()) {
    return;
  }

  // Calculate grid distance from each grid to the nearest wall
  std::vector<double> grid_distance_to_walls = CalculateGridDistanceToWalls();

  for (int32_t y = 0; y < static_cast<int32_t>(height_); y++) {
    for (int32_t x = 0; x < static_cast<int32_t>(width_); x++) {
      const int32_t current = x + (y * width_);
      if (grid_distance_to_walls.at(current) <= potential_grid_width) {
        // Calculate and store potential values
        const unsigned char potential_value = UCHAR_MAX -
            static_cast<unsigned char>(grid_distance_to_walls.at(current) / potential_grid_width *
                                       (kWallValue - kFreeValue));
        if (data_.at(current) < potential_value) {
          data_.at(current) = potential_value;
        }
      }
    }
  }
  potential_width_ = potential_width;
}

/// Calculate grid distance from each grid to the nearest wall
std::vector<double> DistanceMap::CalculateGridDistanceToWalls() {
  // Information about the nearest wall for each grid
  std::vector<int32_t> nearest_wall_indexes;
  nearest_wall_indexes.clear();
  nearest_wall_indexes.resize(height_ * width_);
  // Where there is a wall, store its own grid index
  for (int32_t y = 0; y < static_cast<int32_t>(height_); y++) {
    for (int32_t x = 0; x < static_cast<int32_t>(width_); x++) {
      const int32_t current = x + (y * width_);
      int32_t nearest_wall_index;
      if (data_.at(current) == UCHAR_MAX) {
        nearest_wall_index = current;
      } else {
        nearest_wall_index = kInvalidNearestWall;
      }
      nearest_wall_indexes.at(current) = nearest_wall_index;
    }
  }

  const int32_t kLeft = -1;
  const int32_t kRight = 1;
  const int32_t kUp = width_;
  const int32_t kDown = -width_;
  const int32_t kLeftUp = kLeft + kUp;
  const int32_t kRightUp = kRight + kUp;
  const int32_t kLeftDown = kLeft + kDown;
  const int32_t kRightDown = kRight + kDown;
  // Calculate for the 8 directions: up, down, left, right, and diagonals
  std::vector<int32_t> neighbors = { kLeftDown, kDown, kRightDown, kRight,
                                     kRightUp, kUp, kLeftUp, kLeft };
  std::vector<double> grid_distance_to_walls;
  grid_distance_to_walls.resize(height_ * width_);
  // Scan from bottom left to top right
  for (int32_t y = static_cast<int32_t>(height_ - 1); y >= 0; y--) {
    for (int32_t x = 0; x <= static_cast<int32_t>(width_ - 1); x++) {
      const int32_t current = x + (y * width_);
      double distance = std::numeric_limits<double>::max();
      for (int32_t i = 0; i < static_cast<int32_t>(neighbors.size()); i++) {
        const int32_t neighbor = current + neighbors[i];
        if (neighbor < 0 || neighbor >= static_cast<int32_t>(nearest_wall_indexes.size())) {
          continue;
        }
        UpdateDistanceAndNearestWall(x, y, neighbor, nearest_wall_indexes, distance);
      }
      grid_distance_to_walls.at(current) = distance;
    }
  }

  // Scan from top right to bottom left
  for (int32_t y = 0; y <= static_cast<int32_t>(height_ - 1); y++) {
    for (int32_t x = static_cast<int32_t>(width_ - 1); x >= 0; x--) {
      const int32_t current = x + (y * width_);
      double distance = std::numeric_limits<double>::max();
      for (int32_t i = 0; i < static_cast<int32_t>(neighbors.size()); i++) {
        const int32_t neighbor = current + neighbors[i];
        if (neighbor < 0 || neighbor >= static_cast<int32_t>(nearest_wall_indexes.size())) {
          continue;
        }
        UpdateDistanceAndNearestWall(x, y, neighbor, nearest_wall_indexes, distance);
      }
      grid_distance_to_walls.at(current) = distance;
    }
  }
  return grid_distance_to_walls;
}

/// Update distance to the nearest wall and nearest wall position information
void DistanceMap::UpdateDistanceAndNearestWall(const int32_t x, const int32_t y, const int32_t neighbor_index,
                                               std::vector<int32_t>& nearest_wall_indexes, double& distance) {
  const int32_t current_index = x + (y * width_);
  const int32_t neighbor_nearest_wall_x = nearest_wall_indexes.at(neighbor_index) % width_;
  const int32_t neighbor_nearest_wall_y =
      (nearest_wall_indexes.at(neighbor_index) - neighbor_nearest_wall_x) / width_;
  double tmp_distance = CalculateDistance(neighbor_nearest_wall_x, neighbor_nearest_wall_y, x, y);
  if (tmp_distance < distance) {
    distance = tmp_distance;
    nearest_wall_indexes.at(current_index) = nearest_wall_indexes.at(neighbor_index);
  }
}

void DistanceMap::CalcOccupiedRegion() {
  min_u_ = width_;
  min_v_ = height_;
  max_u_ = 0;
  max_v_ = 0;

  for (size_t v = 0; v < height_; v++) {
    for (size_t u = 0; u < width_; u++) {
      if (data_[u + v * width_] != kFreeValue) {
        if (u < min_u_) {
          min_u_ = u;
        }
        if (u > max_u_) {
          max_u_ = u;
        }

        if (v < min_v_) {
          min_v_ = v;
        }
        if (v > max_v_) {
          max_v_ = v;
        }
      }
    }
  }
}

void DistanceMap::ImageToMap(Pose2d& pose) {
  pose = origin_map_image_ * pose;
}

void DistanceMap::MapToImage(Pose2d& pose) {
  pose = origin_map_image_inverse_ * pose;
}

}  // namespace tmc_pose_2d_lib
