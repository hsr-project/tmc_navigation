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
#ifndef TMC_ASTAR_LIB_ASTAR_NODE_MANAGER_HPP_
#define TMC_ASTAR_LIB_ASTAR_NODE_MANAGER_HPP_

#include <stdint.h>
#include <memory>
#include <vector>
#include "astar_node.hpp"
#include "common.hpp"

namespace tmc_astar_lib {

class IAstarNodeManager {
 public:
  using Ptr = std::shared_ptr<IAstarNodeManager>;
  virtual ~IAstarNodeManager() = default;
  virtual void Initialize() = 0;
  virtual AstarNode* GetNode(const MapIndex& index) = 0;
};

/*
Node management class.
Retrieve the node corresponding to the X, Y coordinates of the grid.

Nodes have a certain size and correspond to all grids of the static map.
Since it is inefficient to always allocate and initialize them, they are handled as follows.
1. Nodes are allocated in a vector, and the elements are incremented by +1 each time a new grid coordinate is referenced.
   To prevent reallocation during size expansion, memory space for the worst-case value (all grids) is pre-allocated.
2. A table to obtain the above index from grid coordinates is prepared for all grids.
3. During initialization, the node vector is cleared, and the entire index table is filled with unused values.
4. When calling GetNode, if the index for the corresponding coordinates is an unused value, the node vector is incremented by +1, and that index is stored.
   If it is a subsequent reference, the corresponding element of the existing node vector is returned.
*/
class AstarNodeManager : public IAstarNodeManager {
 public:
  /// Constructor
  AstarNodeManager(const int32_t width, const int32_t height);
  /// Initialization: Set all nodes to an unreferenced state.
  void Initialize();
  /// Retrieve a pointer to the node corresponding to the specified grid coordinates.
  /// @param [I] index Grid index
  /// @return Pointer to the node corresponding to this grid.
  AstarNode* GetNode(const MapIndex& index);

 private:
  // Width of the management range
  int32_t width_;
  // Height of the management range
  int32_t height_;
  // Holds the actual nodes
  std::vector<AstarNode> node_array_;
  // Obtain the index of node_array_ from grid X, Y coordinates
  std::vector<uint32_t> grid_coord_to_index_map_;
};

}  // namespace tmc_astar_lib

#endif  // TMC_ASTAR_LIB_ASTAR_NODE_MANAGER_HPP_
