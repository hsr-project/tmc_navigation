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
#include <tmc_astar_lib/astar_node_manager.hpp>

namespace {
constexpr int32_t kInvalidIndex = -1;
}  // anonymous namespace

namespace tmc_astar_lib {

/// Constructor
AstarNodeManager::AstarNodeManager(const int32_t width, const int32_t height) : width_(width), height_(height) {
  // The table for coordinates to index is allocated the same size as the static map
  grid_coord_to_index_map_.resize(width * height, kInvalidIndex);

  // The node array reserves the maximum memory space to prevent memory addresses from changing due to resizing
  // Actual memory is consumed only for the amount actually used (however, in DEBUG builds, actual memory is allocated at this point)
  node_array_.reserve(width * height);
}

/// Initialization
void AstarNodeManager::Initialize() {
  node_array_.clear();
  std::fill(grid_coord_to_index_map_.begin(), grid_coord_to_index_map_.end(), kInvalidIndex);
}

/// Get a pointer to the node corresponding to the specified grid coordinates
AstarNode* AstarNodeManager::GetNode(const MapIndex& map_index) {
  if ((map_index.x < 0) || (map_index.x >= width_) || (map_index.y < 0) || (map_index.y >= height_)) {
    CONSOLE_BRIDGE_logError("Specified grid coord is out of range: (%d, %d)", map_index.x, map_index.y);
    throw std::range_error("Specified grid coord is out of range.");
  }

  // Calculate a unique key for the combination of x and y
  const size_t key = map_index.y * width_ + map_index.x;

  int32_t index = grid_coord_to_index_map_[key];
  if (index == kInvalidIndex) {
    // If the coordinates are being referenced for the first time, add the entity
    node_array_.emplace_back(map_index);
    index = node_array_.size() - 1;
    grid_coord_to_index_map_[key] = index;
  }

  return &node_array_[index];
}
}  // namespace tmc_astar_lib
