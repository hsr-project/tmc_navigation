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
#include <tmc_astar_lib/astar_executer.hpp>

#include <limits>
#include <memory>
#include <vector>
#include <boost/optional.hpp>

#include <tmc_astar_lib/astar_node_manager.hpp>
#include <tmc_astar_lib/astar_queue.hpp>

namespace tmc_astar_lib {

/// Constructor
AstarExecuter::AstarExecuter(const int32_t width, const int32_t height) {
  node_manager_ = std::make_shared<AstarNodeManager>(AstarNodeManager(width, height));
}

// Execute A* algorithm
// Specify start and goal uniquely
bool AstarExecuter::ExecuteAstar(
    const IMap::ConstPtr& map,
    const MapIndex& start,
    const MapIndex& goal,
    const int32_t max_cost,
    PoseSeq& path) {
  std::vector<MapIndexWithCost> start_index_with_costs;
  start_index_with_costs.push_back(MapIndexWithCost(start, 0));
  std::vector<MapIndexWithCost> goal_index_with_costs;
  goal_index_with_costs.push_back(MapIndexWithCost(goal, 0));
  return ExecuteAstar(map, start_index_with_costs, goal_index_with_costs, max_cost, path);
}

/// Execute A* algorithm
/// Specify start and goal with multiple candidates
/// Costs can be assigned to each candidate
bool AstarExecuter::ExecuteAstar(
    const IMap::ConstPtr& map,
    const std::vector<MapIndexWithCost>& start_index_with_costs,
    const std::vector<MapIndexWithCost>& goal_index_with_costs,
    const int32_t max_cost,
    PoseSeq& path) {

  // Index of the best goal among those reached
  boost::optional<MapIndex> best_goal_index = boost::none;
  // Total cost of the best goal among those reached
  int32_t best_goal_cost = std::numeric_limits<int32_t>::max();

  node_manager_->Initialize();
  AstarQueue::Ptr queue = std::make_shared<AstarQueue>(AstarQueue());
  queue->Initialize(max_cost);
  // Enqueue start candidate points
  for (const MapIndexWithCost& start : start_index_with_costs) {
    if (map->IsPassable(start.index)) {
      AstarNode* const start_node = node_manager_->GetNode(start.index);
      start_node->Update(nullptr, start.cost, 0, true);
      queue->Push(start_node);
    }
  }
  while (true) {
    // Dequeue the node with the smallest cost from the start
    AstarNode* const current_node = queue->Pop();
    // If a goal candidate is reached and no better result is expected, terminate the search
    if (best_goal_index && (current_node == NULL || current_node->total_cost() >= best_goal_cost)) {
      break;
    }
    // If the queue becomes empty (=no room for exploration within max_cost), give up
    if (current_node == NULL) {
      break;
    }
    // Discard if already explored due to design allowing multiple pushes of the same node
    if (current_node->is_closed()) {
      continue;
    }
    // Check if a goal candidate has been reached
    for (const MapIndexWithCost& goal : goal_index_with_costs) {
      if (current_node->index() == goal.index) {
        // Add the last step to the cost so far to calculate the final cost
        const int32_t candidate_final_cost = current_node->total_cost() + goal.cost;
        // If the current cost is the lowest, record the reached point as the final candidate
        if (candidate_final_cost < best_goal_cost) {
          best_goal_index = goal.index;
          best_goal_cost = candidate_final_cost;
        }
      }
    }
    // Get the next point
    map->GetNextNodes(queue, node_manager_, current_node, max_cost);
    // Mark the current node as explored
    current_node->Close();
  }
  if (best_goal_index) {
    ConvertToPath(map, best_goal_index.get(), path);
    return true;
  }
  return false;
}

/// Generate the optimal path from the nodes resulting from A* execution
void AstarExecuter::ConvertToPath(
    const IMap::ConstPtr& map,
    const MapIndex& goal_index, PoseSeq& path) {
  AstarNode* const goal_node = node_manager_->GetNode(goal_index);
  const int32_t path_length = goal_node->total_step() + 1;
  path.resize(path_length);

  // Cannot trace from the start side due to branching
  // Trace from the goal to the start while appending the output path from the back
  AstarNode* current_node = goal_node;
  for (int32_t index = path_length - 1; index >= 0; --index) {
    map->IndexToPose(current_node->index(), path[index]);
    // Terminate when reaching the start
    if (current_node->total_step() == 0) {
      break;
    }
    // Step back one node
    current_node = current_node->parent();
  }
}
}  // namespace tmc_astar_lib
