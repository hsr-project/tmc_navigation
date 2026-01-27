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
#ifndef TMC_ASTAR_LIB_LAYERED_COST_MAP_HPP_
#define TMC_ASTAR_LIB_LAYERED_COST_MAP_HPP_
#include <climits>
#include <stdint.h>
#include <limits>
#include <memory>
#include <vector>
#include <console_bridge/console.h>

#include "common.hpp"
#include "cost_corrector/cost_corrector.hpp"
#include "map.hpp"
#include "node_direction.hpp"

namespace { // NOLINT
/// Parameter default value
constexpr double kExclusiveSizeDefault = 0.2;
constexpr double kPotentialSizeDefault = 0.07;
constexpr double kWallThresholdDefault = 3.0;
constexpr double kCostFactorDefault = 150.0;
constexpr int32_t kSingleCostDefault = 50;
constexpr int32_t kDiagonalCostDefault = 71;
}  // anonymous namespace

namespace tmc_astar_lib {

// Search direction
struct SearchDirectionInfo {
  int32_t offset_x;
  int32_t offset_y;
  NodeDirection direction;
  int32_t cost;
};

/// Overlay static map and dynamic map, and refer to both in the same coordinate system (static map's coordinate system)
/// The search logic performs exploration by repeatedly calling GetNextNodes provided by this class
/// The load of GetNextNodes and related processes greatly affects the overall speed of exploration
/// When making changes to this class, consider the impact on processing load
class LayeredCostMap : public IMap {
 public:
  using Ptr = std::shared_ptr<LayeredCostMap>;
  using ConstPtr = std::shared_ptr<LayeredCostMap const>;
  /// Parameters
  struct Parameter {
    Parameter() {}
    Parameter(const double in_exclusive_size, const double in_potential_size,
              const double in_wall_threshold, const double in_cost_factor,
              const int32_t in_cost_unknown, const int32_t in_single_cost, const int32_t in_diagonal_cost) :
        exclusive_size(in_exclusive_size), potential_size(in_potential_size),
        wall_threshold(in_wall_threshold), cost_factor(in_cost_factor),
        cost_unknown(in_cost_unknown), single_cost(in_single_cost), diagonal_cost(in_diagonal_cost) {
      if (exclusive_size < 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'exclusive_size' is invalid. Use default value.");
        exclusive_size = kExclusiveSizeDefault;
      }
      if (potential_size < 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'potential_size' is invalid. Use default value.");
        potential_size = kPotentialSizeDefault;
      }
      if (wall_threshold <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'wall_threshold' is invalid. Use default value.");
        wall_threshold = kWallThresholdDefault;
      }
      if (exclusive_size - wall_threshold > std::numeric_limits<double>::epsilon() ||
          exclusive_size + potential_size - wall_threshold > std::numeric_limits<double>::epsilon()) {
        CONSOLE_BRIDGE_logWarn("Value of 'wall_threshold' must be greater than 'exclusive_size + potential_size'. "
                               "Use default values for all three parameters.");
        exclusive_size = kExclusiveSizeDefault;
        potential_size = kPotentialSizeDefault;
        wall_threshold = kWallThresholdDefault;
      }

      if (cost_factor < 0) {
        CONSOLE_BRIDGE_logWarn("Value of 'cost_factor' is invalid. Use default value.");
        cost_factor = kCostFactorDefault;
      }
      if (single_cost <= 0) {
        CONSOLE_BRIDGE_logWarn("Value of 'single_cost' is invalid. Use default value.");
        single_cost = kSingleCostDefault;
      }
      if (diagonal_cost <= 0) {
        CONSOLE_BRIDGE_logWarn("Value of 'diagonal_cost' is invalid. Use default value.");
        diagonal_cost = kDiagonalCostDefault;
      }
    }
    // Forbidden area size [m]
    double exclusive_size;
    // Potential area size [m]
    double potential_size;
    // Distance affected by the wall [m]
    double wall_threshold;
    // Cost value indicating unknown area
    int32_t cost_unknown;
    // Cost estimation coefficient
    double cost_factor;
    // Movement cost between adjacent grids
    int32_t single_cost;
    // Movement cost for diagonal direction grids
    int32_t diagonal_cost;
  };
  /// Constructor
  explicit LayeredCostMap(const Parameter& param, const CostMapPtr& static_map);
  /// Initialization
  void Initialize(const CostMapPtr& static_map);
  /// Obtain adjacent nodes that can be moved to from the specified node
  void GetNextNodes(AstarQueue::Ptr& queue, IAstarNodeManager::Ptr& node_manager, AstarNode* const current_node,
                    const int32_t max_cost) const;
  /// Get index from coordinates
  void PoseToIndex(const Pose2d& pose, MapIndex& index) const;
  /// Get index within range from coordinates and range
  void PoseToIndexes(const Pose2d& pose, const double range, std::vector<MapIndex>& indexes) const;
  /// Get coordinates from index
  void IndexToPose(const MapIndex& index, Pose2d& pose) const;
  /// Is the specified index passable?
  bool IsPassable(const MapIndex& index) const;
  /// Maximum cost estimation
  int32_t EstimateMaxCost(const Pose2d& start, const Pose2d& goal) const;
  /// Dynamic map setting
  void SetDynamicMap(const CostMapPtr& dynamic_map, const Pose2d& dyamic_map_origin);
  /// Check if the specified grid coordinates are within the range of the static map
  bool IsOnMap(const MapIndex& index) const;
  // Set cost_corrector
  void SetCostCollectors(const std::vector<ICostCorrector::Ptr>& cost_correctors) {
    cost_correctors_ = cost_correctors;
  }

  // getter
  int32_t width() const { return width_; }
  int32_t height() const { return height_; }
  uint8_t static_map_occupancy_threshold() const { return static_map_occupancy_threshold_; }

 private:
  /// Get static cost of specified grid coordinates
  int32_t GetStaticMapCost(const MapIndex& index) const;
  /// Get dynamic cost of specified grid coordinates
  int32_t GetDynamicMapCost(const MapIndex& index) const;
  // Generate cost conversion table considering potential
  void CreatePotentialCostTable();
  // Generate search direction table
  void CreateSearchDirInfo();
  // Calculate the coverage range of the dynamic map at the grid coordinates of the static map
  void CalcDynamicMapRange(const Pose2d& origin, const int32_t width, const int32_t height);
  /// Convert to cost value considering potential
  int32_t ApplyPotentialSlope(const int32_t cost_value) const;

  // Search direction table
  std::vector<SearchDirectionInfo> search_dir_info_[static_cast<int32_t>(NodeDirection::DIR_Max)];
  // Dynamic map
  CostMapPtr dynamic_map_;
  // Inverse transformation of the relative origin of the dynamic map based on the static map origin
  Pose2d dynamic_map_origin_inverse_;
  // Threshold for forbidden area of static map
  uint8_t static_map_occupancy_threshold_;
  // Cost conversion table considering potential
  std::array<uint8_t, UCHAR_MAX + 1> potential_table_;
  // Static map with applied potential
  std::vector<uint8_t> static_cost_map_;
  // Map size
  const int32_t width_;
  const int32_t height_;
  // Map resolution
  const double resolution_;
  // Coverage range of the dynamic map at the grid coordinates of the static map
  int32_t dynamic_map_range_xmin_;
  int32_t dynamic_map_range_xmax_;
  int32_t dynamic_map_range_ymin_;
  int32_t dynamic_map_range_ymax_;

  std::vector<ICostCorrector::Ptr> cost_correctors_;
  // Parameters
  Parameter param_;
};
}  // namespace tmc_astar_lib

#endif  // TMC_ASTAR_LIB_LAYERED_COST_MAP_HPP_
