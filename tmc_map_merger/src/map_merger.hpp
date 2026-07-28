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
#ifndef TMC_MAP_MERGER_MAP_MERGER_HPP_
#define TMC_MAP_MERGER_MAP_MERGER_HPP_
#include <stdint.h>

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "map.hpp"
#include "map_projector.hpp"

namespace tmc_map_merger {

template<typename Memory, typename Type = int8_t>
struct MemoryAdapter {
  typedef Type DataType;
  MemoryAdapter(const rclcpp::Time& t, const typename Memory::Option& op)
      : time(t), option(op) {
  }
  void Get(Type& dst, const Memory& m) const {
    dst = m.Get(time, option);
  }
  void Update(Memory& dst, Type src) const {
    dst.Update(src, time, option);
  }
  void Reset(Memory& dst) const {
    dst.Reset(time, option);
  }
  const typename Memory::Option& option;
  rclcpp::Time time;
};

/// @brief Base class for map merging
class MapMerger {
 public:
  typedef std::shared_ptr<MapMerger> Ptr;
  /// @brief Constructor
  MapMerger(uint32_t width, uint32_t height, double resolution) {
    map_.info.width = width;
    map_.info.height = height;
    map_.info.resolution = resolution;
    map_.info.origin.orientation = geometry_msgs::msg::Quaternion();
    map_.info.origin.orientation.w = 1.0;
    SimpleMapOperator op(map_);
    op.Reset();
  }

  /// @brief Destructor
  virtual ~MapMerger() {
  }

  /// @brief Clear
  virtual void Clear() {
    SimpleMapOperator op(map_);
    op.Reset();
  }

  /// @brief Output of the map
  virtual const Map& GetMap() {
    return map_;
  }

  /// @brief Input of the map
  virtual void Merge(const Map& map) = 0;

  /// @brief Update position and time
  virtual void Update(const geometry_msgs::msg::PoseStamped& origin) = 0;

 protected:
  Map map_;
};


/// @brief Simple merge
/// Merge that only projects the input map according to UpdateStrategy
/// The map to be maintained has a fixed direction and aligns on a grid unit
template<typename UpdateStrategy>
class SimpleMapMerger : public MapMerger {
 public:
  /// @brief Constructor
  SimpleMapMerger(uint32_t width, uint32_t height, double resolution)
      : MapMerger(width, height, resolution) {
  }

  /// @brief Destructor
  virtual ~SimpleMapMerger() {
  }

  /// @brief Input of the map
  virtual void Merge(const Map& map) {
    Project(ConstSimpleMapOperator(map),
            MapOperator<UpdateStrategy>(map_));
  }

  /// @brief Update position and time
  virtual void Update(const geometry_msgs::msg::PoseStamped& origin) {
    MapInfo pre_info = map_.info;
    map_.info.origin = origin.pose;
    map_.info.origin.orientation = geometry_msgs::msg::Quaternion();
    map_.info.origin.orientation.w = 1.0;
    // Fix the rotation and align it to a grid position
    map_.info.origin.position.x = \
        ::round(static_cast<double>(origin.pose.position.x / map_.info.resolution)) * map_.info.resolution
        - map_.info.width * map_.info.resolution / 2.0;
    map_.info.origin.position.y = \
        ::round(static_cast<double>(origin.pose.position.y / map_.info.resolution)) * map_.info.resolution
        - map_.info.height * map_.info.resolution / 2.0;
    /// Copy as the origin is moved
    ProjectGrid(ConstSimpleMapOperator(map_, pre_info),
                SimpleMapOperator(map_));
  }
};

/// @brief Map merge with memory
/// Memory is a class that implements memory retention
/// For optimization (to avoid function pointer calls)
/// Implement polymorphism using templates
/// Implement the following methods
/// Get, Update, Reset
template<typename Memory>
class MemoryMapMerger : public MapMerger {
  typedef AnyMap<Memory> MemoryMap;
  typedef MapOperator<BasicAdapter<Memory>, MemoryMap> SimpleMemoryMapOperator;
  typedef MapOperator<MemoryAdapter<Memory>, MemoryMap> MemoryMapOperator;

 public:
  /// @brief Constructor
  MemoryMapMerger(uint32_t width, uint32_t height, double resolution)
      : MapMerger(width, height, resolution), memory_(map_.info) {
    // Initialization
    MemoryMapOperator op(memory_, map_.info, MemoryAdapter<Memory>(rclcpp::Time(0), memory_option_));
    op.Reset();
  }

  /// @brief Constructor
  MemoryMapMerger(uint32_t width, uint32_t height, double resolution, const typename Memory::Option& option)
      : MapMerger(width, height, resolution), memory_(map_.info), memory_option_(option) {
    // Initialization
    MemoryMapOperator op(memory_, map_.info, MemoryAdapter<Memory>(rclcpp::Time(0), memory_option_));
    op.Reset();
  }

  /// @brief Destructor
  virtual ~MemoryMapMerger() {
  }

  /// @brief Clear
  virtual void Clear() {
    MapMerger::Clear();
    MemoryMapOperator op(memory_, map_.info, MemoryAdapter<Memory>(rclcpp::Time(0), memory_option_));
    op.Reset();
  }

  /// @brief Input of the map
  virtual void Merge(const Map& map) {
    Project(ConstSimpleMapOperator(map),
            MemoryMapOperator(memory_, map_.info, MemoryAdapter<Memory>(map.header.stamp, memory_option_)));
  }

  /// @brief Update position and time
  virtual void Update(const geometry_msgs::msg::PoseStamped& origin) {
    MapInfo pre_info = map_.info;
    map_.info.origin = origin.pose;
    // Fix the rotation and align it to a grid position
    map_.info.origin.orientation = geometry_msgs::msg::Quaternion();
    map_.info.origin.orientation.w = 1.0;
    map_.info.origin.position.x = \
        ::round(static_cast<double>(origin.pose.position.x / map_.info.resolution)) * map_.info.resolution
        - map_.info.width * map_.info.resolution / 2.0;
    map_.info.origin.position.y = \
        ::round(static_cast<double>(origin.pose.position.y / map_.info.resolution)) * map_.info.resolution
        - map_.info.height * map_.info.resolution / 2.0;
    /// Copy as the origin is moved
    ProjectGrid(SimpleMemoryMapOperator(memory_, pre_info),
                SimpleMemoryMapOperator(memory_, map_.info));
    // Output to the map
    MemoryMapOperator op(memory_, map_.info, MemoryAdapter<Memory>(origin.header.stamp, memory_option_));
    for (size_t index = 0; index < memory_.data.size(); ++index) {
      map_.data[index] = op.Get(index);
    }
  }

 protected:
  /// @brief Array of memory
  MemoryMap memory_;
  /// @brief Options for memory operations
  typename Memory::Option memory_option_;
};

class MapMergerFactory {
 public:
  static MapMerger::Ptr Create(const std::map<std::string, rclcpp::Parameter>& parameters);
};

}  // namespace tmc_map_merger

#endif
