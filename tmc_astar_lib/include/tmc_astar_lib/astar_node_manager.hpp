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

Nodes have a considerable size, and nodes corresponding to all grids of the static map
are inefficient to always allocate and initialize, so they are handled as follows.
1. Nodes are allocated with a vector, and the elements are incremented by +1 each time new grid coordinates are referenced.
   To prevent relocation during size expansion, only the memory area is allocated for the worst-case (all grids).
2. Prepare a table to obtain the above index from grid coordinates for all grids.
3. During initialization, clear the node vector and fill the entire index table with unused values.
4. When GetNode is called, if the index of the corresponding coordinates is an unused value, increment the node vector by +1 and store that index.
   If it is a subsequent reference, return the corresponding element of the existing node vector.
*/
class AstarNodeManager : public IAstarNodeManager {
 public:
  /// Constructor
  AstarNodeManager(const int32_t width, const int32_t height);
  /// Initialization Set all nodes to an unreferenced state
  void Initialize();
  /// Obtain a pointer to the node corresponding to the specified grid coordinates
  /// @param [I] index Grid index
  /// @return Pointer to the node corresponding to this grid
  AstarNode* GetNode(const MapIndex& index);

 private:
  // Width of the management range
  int32_t width_;
  // Height of the management range
  int32_t height_;
  // Holds the entity of the node
  std::vector<AstarNode> node_array_;
  // Obtain the index of node_array_ from grid X, Y coordinates
  std::vector<uint32_t> grid_coord_to_index_map_;
};

}  // namespace tmc_astar_lib

#endif  // TMC_ASTAR_LIB_ASTAR_NODE_MANAGER_HPP_
