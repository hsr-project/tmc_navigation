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
#include <tmc_astar_lib/layered_cost_map.hpp>

#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>

namespace {
// Resolution of potential data
constexpr double kDepth = 255.0;
enum class SearchRange {
  Range_45 = 1,   // Search in three directions: front + 45° to the left and right based on the direction of travel
  Range_90 = 2,   // Search in five directions: front + 90° to the left and right based on the direction of travel
  Range_135 = 3,  // Search in seven directions: front + 135° to the left and right based on the direction of travel
  Range_All = 4   // Search in all directions
};
// Search range for the next grid relative to the direction of travel
constexpr SearchRange kSearchRange = SearchRange::Range_45;
}  // anonymous namespace

namespace tmc_astar_lib {

/// Constructor
LayeredCostMap::LayeredCostMap(const Parameter& param, const CostMapPtr& static_map) :
    width_(static_map->width()), height_(static_map->height()), resolution_(static_map->resolution()), param_(param) {
  Initialize(static_map);
}

void LayeredCostMap::Initialize(const CostMapPtr& static_map) {
  // Generate a cost conversion table considering potential
  CreatePotentialCostTable();
  // Generate a search direction table
  CreateSearchDirInfo();

  // Convert to a map with applied potential
  static_cost_map_.resize(width_ * height_);
  for (int32_t y = 0; y < height_; ++y) {
    for (int32_t x = 0; x < width_; ++x) {
      uint8_t value;
      static_map->GetValueAt(x, y, value);
      static_cost_map_[y * width_ + x] = ApplyPotentialSlope(value);
    }
  }
  // Calculate the threshold for restricted areas in the static map
  static_map_occupancy_threshold_ = static_cast<uint8_t>(
      (1.0 - param_.exclusive_size / param_.wall_threshold) * kWallValue);
}


// Generate a cost conversion table considering potential
void LayeredCostMap::CreatePotentialCostTable() {
  const int32_t dfree = static_cast<int32_t>(std::round(
    (1.0 - (param_.exclusive_size + param_.potential_size) / param_.wall_threshold) * kDepth));
  const int32_t dwall =
    static_cast<int32_t>(std::round(
    (1.0 - param_.exclusive_size / param_.wall_threshold) * kDepth));
  for (int32_t i = 0; i < UCHAR_MAX + 1; i++) {
    if (i >= 0 && i < dwall && i != param_.cost_unknown) {
      if (i < dfree) {
        potential_table_[i] = 0;
      } else {
        potential_table_[i] = static_cast<int8_t>(std::round(
          static_cast<double>(i - dfree) / static_cast<double>(dwall) * (kDepth / 2)));
      }
    } else {
      potential_table_[i] = kWallValue;
    }
  }
}

// Generate a search direction table
void LayeredCostMap::CreateSearchDirInfo() {
  // Search in the forward direction and 45° * kSearchRange to the left and right
  const int32_t offset_left = static_cast<int32_t>(kSearchRange);    // 45° * kSearchRange to the left
  const int32_t offset_right = -static_cast<int32_t>(kSearchRange);  // 45° * kSearchRange to the right
  const int32_t direction_num = static_cast<int32_t>(NodeDirection::DIR_Max) - 1;

  for (int32_t i = static_cast<int32_t>(NodeDirection::DIR_0); i <= direction_num; ++i) {
    for (int32_t offset = offset_right; offset <= offset_left; ++offset) {
      const int32_t dir_index =
          (i + offset - static_cast<int32_t>(NodeDirection::DIR_0) + direction_num) % direction_num +
          static_cast<int32_t>(NodeDirection::DIR_0);

      SearchDirectionInfo info;
      GetOffsetFromDirection(static_cast<NodeDirection>(dir_index), info.offset_x, info.offset_y);
      info.direction = static_cast<NodeDirection>(dir_index);
      if (info.offset_x != 0 && info.offset_y != 0) {
        info.cost = param_.diagonal_cost;
      } else {
        info.cost = param_.single_cost;
      }
      search_dir_info_[i].push_back(info);
    }
  }
  // Since the starting point has no specific direction of travel and can proceed in any direction, search in all directions
  for (int32_t dir_index = static_cast<int32_t>(NodeDirection::DIR_0); dir_index <= direction_num; ++dir_index) {
    SearchDirectionInfo info;
    GetOffsetFromDirection(static_cast<NodeDirection>(dir_index), info.offset_x, info.offset_y);
    info.direction = static_cast<NodeDirection>(dir_index);
    if (info.offset_x != 0 && info.offset_y != 0) {
      info.cost = param_.diagonal_cost;
    } else {
      info.cost = param_.single_cost;
    }
    search_dir_info_[static_cast<int32_t>(NodeDirection::DIR_None)].push_back(info);
  }
}

/// Dynamic map settings
void LayeredCostMap::SetDynamicMap(const CostMapPtr& dynamic_map, const Pose2d& dynamic_map_origin) {
  dynamic_map_ = dynamic_map;
  // Inverse transformation of the dynamic map origin
  // To avoid performance degradation from conversion during each cost retrieval process, keep the inverse-transformed data
  dynamic_map_origin_inverse_ = dynamic_map_origin.Inverse();
  // Pre-calculate the range on the static map that could fall within the dynamic map's range
  // To reduce the cost retrieval load in areas clearly outside the dynamic map's range
  CalcDynamicMapRange(dynamic_map_origin, dynamic_map->width(), dynamic_map->height());
}

// Calculate the coverage range of the dynamic map on the grid coordinates of the static map
void LayeredCostMap::CalcDynamicMapRange(const Pose2d& origin, const int32_t width, const int32_t height) {
  const double width_cos = width * dynamic_map_->resolution() * cos(origin.theta());
  const double width_sin = width * dynamic_map_->resolution() *  sin(origin.theta());
  const double height_cos = height * dynamic_map_->resolution() * cos(origin.theta());
  const double height_sin = height * dynamic_map_->resolution() * sin(origin.theta());

  // Coordinates of the four corners
  const Point2d v_a = origin.point();
  const Point2d v_b = Point2d(v_a.x() + width_cos, v_a.y() + width_sin);
  const Point2d v_c = Point2d(v_a.x() + height_sin, v_a.y() + height_cos);;
  const Point2d v_d = Point2d(v_c.x() + width_cos, v_c.y() + width_sin);;

  dynamic_map_range_xmin_ = static_cast<int32_t>(
      std::floor(std::min({v_a.x(), v_b.x(), v_c.x(), v_d.x()}) / resolution_));
  dynamic_map_range_ymin_ = static_cast<int32_t>(
      std::floor(std::min({v_a.y(), v_b.y(), v_c.y(), v_d.y()}) / resolution_));
  dynamic_map_range_xmax_ = static_cast<int32_t>(
      std::floor(std::max({v_a.x(), v_b.x(), v_c.x(), v_d.x()}) / resolution_)) - 1;
  dynamic_map_range_ymax_ = static_cast<int32_t>(
      std::floor(std::max({v_a.y(), v_b.y(), v_c.y(), v_d.y()}) / resolution_)) - 1;
}

/// Retrieve index from coordinates
void LayeredCostMap::PoseToIndex(const Pose2d& pose, MapIndex& index) const {
  // Assume the map origin aligns with the grid corner and round towards negative infinity
  index.x = static_cast<int32_t>(std::floor(pose.x() / resolution_));
  index.y = static_cast<int32_t>(std::floor(pose.y() / resolution_));
}

/// Retrieve indices within range from coordinates and range
void LayeredCostMap::PoseToIndexes(const Pose2d& pose, const double range, std::vector<MapIndex>& indexes) const {
  if (range < std::numeric_limits<double>::epsilon()) {
    // If no range is specified, output only the grid to which the specified pose belongs
    MapIndex index;
    PoseToIndex(pose, index);
    indexes.push_back(index);
    return;
  }
  // Output grids overlapping with a square centered on the specified coordinates
  const int32_t x_min = static_cast<int32_t>(std::floor((pose.x() - range) / resolution_));
  const int32_t y_min = static_cast<int32_t>(std::floor((pose.y() - range) / resolution_));
  const int32_t x_max = static_cast<int32_t>(std::floor((pose.x() + range) / resolution_));
  const int32_t y_max = static_cast<int32_t>(std::floor((pose.y() + range) / resolution_));
  for (int32_t i = x_min; i <= x_max; ++i) {
    for (int32_t j = y_min; j <= y_max; ++j) {
      MapIndex index(i, j);
      if (IsOnMap(index)) {
        indexes.push_back(index);
      }
    }
  }
}

/// Retrieve coordinates from index
void LayeredCostMap::IndexToPose(const MapIndex& index, Pose2d& pose) const {
  // Add +0.5 before conversion to point to the center of the grid
  pose.set_x((static_cast<double>(index.x) + 0.5) * resolution_);
  pose.set_y((static_cast<double>(index.y) + 0.5) * resolution_);
}

/// Check if the specified index is passable
bool LayeredCostMap::IsPassable(const MapIndex& index) const {
  return (GetStaticMapCost(index) != kWallValue) && (GetDynamicMapCost(index) != kWallValue);
}

/// Maximum cost estimation
int32_t LayeredCostMap::EstimateMaxCost(const Pose2d& start, const Pose2d& goal) const {
  /// The maximum cost is the sum of the grid X-coordinate difference and grid Y-coordinate difference between the start and goal, multiplied by a coefficient
  MapIndex start_index;
  MapIndex goal_index;
  PoseToIndex(start, start_index);
  PoseToIndex(goal, goal_index);
  int32_t diff = abs(start_index.x - goal_index.x) + abs(start_index.y - goal_index.y);
  return static_cast<int32_t>(std::round(param_.cost_factor * diff));
}

/// Retrieve adjacent nodes that can be moved to from the specified node
void LayeredCostMap::GetNextNodes(IAstarQueue::Ptr& queue, IAstarNodeManager::Ptr& node_manager,
                                  AstarNode* const current_node, const int32_t max_cost) const {
  // Search adjacent grids based on the current node's direction of travel
  // The direction of travel for the node is set in the attached information
  const std::vector<SearchDirectionInfo>& search_directions =
      search_dir_info_[current_node->additional_info()];
  for (const SearchDirectionInfo& info : search_directions) {
    const MapIndex index(current_node->index().x + info.offset_x,
                         current_node->index().y + info.offset_y);
    const int32_t static_cost = GetStaticMapCost(index);
    const int32_t dynamic_cost = GetDynamicMapCost(index);
    if (static_cost != kWallValue && dynamic_cost != kWallValue) {
      AstarNode* const next_node = node_manager->GetNode(index);
      // If the destination already has a better cost than the current node, skip this path as it cannot be a candidate
      if (current_node->total_cost() >= next_node->total_cost()) {
        continue;
      }
      // Basic cost based on the direction of travel
      int32_t step_cost = info.cost;
      // For locations where static and dynamic obstacles overlap, take the larger value
      step_cost += std::max(static_cost, dynamic_cost);
      // Execute correction process
      const ICostCorrector::GetAdditionalCostParams corrector_param(
          current_node, index, static_cast<int32_t>(info.direction),
          static_cost, dynamic_cost, static_map_occupancy_threshold_);
      for (const ICostCorrector::Ptr& cost_corrector : cost_correctors_) {
        step_cost += cost_corrector->GetAdditionalCost(corrector_param);
      }

      // To prevent infinite loops at points with negative costs, limit the lower bound to 0
      step_cost = std::max(step_cost, 0);

      const int32_t total_cost = current_node->total_cost() + step_cost;
      if (total_cost <= max_cost) {
        // If the current cost is advantageous, update and enqueue it
        if (total_cost < next_node->total_cost()) {
          // Set the direction of travel for the node in the attached information
          next_node->Update(current_node, total_cost, current_node->total_step() + 1, true,
                            static_cast<int32_t>(info.direction));
          queue->Push(next_node);
        }
      }
    }
  }
}

/// Check if the specified index is within the map's range
bool LayeredCostMap::IsOnMap(const MapIndex& index) const {
  return index.x >= 0 && index.x < width_ && index.y >= 0 && index.y < height_;
}

/// Retrieve the static cost of the specified grid coordinates
int32_t LayeredCostMap::GetStaticMapCost(const MapIndex& index) const {
  if (!IsOnMap(index)) {
    return kWallValue;
  } else {
    return static_cost_map_[index.y * width_ + index.x];
  }
}

/// Retrieve the dynamic cost of the specified grid coordinates
int32_t LayeredCostMap::GetDynamicMapCost(const MapIndex& index) const {
  if (index.x < dynamic_map_range_xmin_ || index.x > dynamic_map_range_xmax_ ||
      index.y < dynamic_map_range_ymin_ || index.y > dynamic_map_range_ymax_) {
    // Return free if outside the range of the dynamic map
    return 0;
  }
  const Pose2d coordinate_on_static(index.x * resolution_, index.y * resolution_, 0.0);
  const Pose2d coordinate_on_dynamic = dynamic_map_origin_inverse_ * coordinate_on_static;
  // Determine the grid coordinates on the dynamic map
  const int32_t x_on_dynamic = static_cast<int32_t>(
      std::floor(coordinate_on_dynamic.x() / dynamic_map_->resolution()));
  const int32_t y_on_dynamic = static_cast<int32_t>(
      std::floor(coordinate_on_dynamic.y() / dynamic_map_->resolution()));
  uint8_t value;
  dynamic_map_->GetValueAt(x_on_dynamic, y_on_dynamic, value);
  if (value <= kFreeGrid) {
    return 0;
  }
  return static_cast<int32_t>(value);
}

/// Convert to a cost value considering potential
int32_t LayeredCostMap::ApplyPotentialSlope(const int32_t cost_value) const {
  return potential_table_[cost_value];
}

}  // namespace tmc_astar_lib
