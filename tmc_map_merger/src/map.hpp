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
#ifndef TMC_MAP_MERGER_MAP_HPP_
#define TMC_MAP_MERGER_MAP_HPP_

#include <cmath>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

namespace { // NOLINT

/// @brief Unobserved
const int8_t kUnknown = -1;
/// @brief Unoccupied
const int8_t kFree = 0;

/// @brief Directly calculate Yaw from geometry_msgs::msg::Quaternion
double GetYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z));
}

/// @brief round (std::round is not available before C++11)
double round(double number) {
  return number < 0.0 ? std::ceil(number - 0.5) : std::floor(number + 0.5);
}

}  // anonymous namespace

namespace tmc_map_merger {
typedef nav_msgs::msg::OccupancyGrid Map;
typedef nav_msgs::msg::MapMetaData MapInfo;

/// @brief Map that can hold a one-dimensional array of any type T
/// Holds data and info (of type MapInfo) as members
template<typename T>
struct AnyMap {
  explicit AnyMap(const MapInfo& i)
      : info(i) {}
  MapInfo info;
  std::vector<T> data;
};

/// @brief Adapter that does nothing in particular
template<typename T>
struct BasicAdapter {
  /// @brief Internal type
  /// Declaration of DataType is mandatory for use by MapOperator
  typedef T DataType;
  /// @brief Extract DataType type as V type
  template<typename V>
  void Get(V& dst, const T& src) const {
    dst = src;
  }
  /// @brief Store V type as DataType type
  template<typename V>
  void Update(T& dst, const V src) const {
    dst = src;
  }
  /// @brief Reset DataType type
  void Reset(T& dst) const {
    dst = T();
  }
};

/// @brief Adapter for Map(int8_t). Resets to Unknown
struct BasicMapAdapter {
  typedef int8_t DataType;
  template<typename V>
  void Get(V& dst, const int8_t& src) const {
    dst = src;
  }
  template<typename V>
  void Update(int8_t& dst, const V src) const {
    dst = src;
  }
  void Reset(int8_t& dst) const {
    dst = kUnknown;
  }
};

/// @brief Adapter for Map(int8_t). Updates if it's the maximum value.
struct UpdateIfGreaterMapAdapter {
  typedef int8_t DataType;
  template<typename V>
  void Get(V& dst, const int8_t& src) const {
    dst = src;
  }
  template<typename V>
  void Update(int8_t& dst, const V src) const {
    if (src > dst) {
      dst = src;
    }
  }
  void Reset(int8_t& dst) const {
    dst = kUnknown;
  }
};

/// @brief Class that enables access to Map of MapType using Adapter
/// Class with implemented methods: GetInfo, Get, Update, Reset
template<typename Adapter = BasicMapAdapter,
         typename MapType = Map>
struct MapOperator {
 public:
  /// @brief Constructor
  explicit MapOperator(MapType& target)
      : map(target), info(target.info) {}
  /// @brief Constructor
  MapOperator(MapType& target, MapInfo& new_info)
      : map(target), info(new_info) {}
  /// @brief Constructor
  MapOperator(MapType& target, const Adapter& s)
      : map(target), info(target.info), adapter(s) {}
  /// @brief Constructor
  MapOperator(MapType& target, MapInfo& new_info, const Adapter& s)
      : map(target), info(new_info), adapter(s) {}
  /// @brief Retrieve information
  const MapInfo& GetInfo() const {
    return info;
  }
  /// @brief Retrieve (if no type is specified, extract as DataType type)
  typename Adapter::DataType Get(size_t index) const {
    typename Adapter::DataType v;
    adapter.Get(v, map.data[index]);
    return v;
  }
  /// @brief Retrieve (with type specification)
  template<typename V>
  V Get(size_t index) const {
    V v;
    adapter.Get(v, map.data[index]);
    return v;
  }
  /// @brief Update
  template<typename V>
  void Update(size_t index, const V data) const {
    adapter.Update(map.data[index], data);
  }
  /// @brief Initialize
  void Reset(size_t index) const {
    adapter.Reset(map.data[index]);
  }
  /// @brief Full initialization
  void Reset() const {
    map.data.resize(info.width * info.height);
    for (size_t index = 0; index < info.width * info.height; ++index) {
      Reset(index);
    }
  }

 private:
  MapType& map;
  const MapInfo& info;
  Adapter adapter;
};

typedef MapOperator<> SimpleMapOperator;
typedef MapOperator<BasicMapAdapter, const Map> ConstSimpleMapOperator;

}  // namespace tmc_map_merger

#endif
