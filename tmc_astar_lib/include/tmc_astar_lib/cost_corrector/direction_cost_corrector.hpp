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
#ifndef TMC_ASTAR_LIB_DIRECTION_COST_CORRECTOR_HPP_
#define TMC_ASTAR_LIB_DIRECTION_COST_CORRECTOR_HPP_

#include "../node_direction.hpp"
#include "cost_corrector.hpp"
/*
Direction Cost Adjustment

Overview:
Reduce costs when moving in a zigzag pattern at angles less than 45°, such as north, north, northeast, north, north, northeast...

Background:
The A* algorithm can only draw paths in 45° increments,
resulting in paths composed of combinations of 0°, 90°, and 45°, where turning points do not affect the cost.
For example, the following patterns have equal costs, and the resulting path is undefined.

①                    ②                    ③
□□□□□□□□□□　□□□□□□□□□□　□□□□□□□□□□
□□□□□□□Ｇ□□　□□□□□□□Ｇ□□　□□□□□□□Ｇ□□
□□□□□□□│□□　□□□□□□／□□□　□□□□□□□／□□
□□□□□□□│□□　□□□□□／□□□□　□□□□□□│□□□
□□□□□□□│□□　□□□□／□□□□□　□□□□□□／□□□
□□□□□□□│□□　□□□／□□□□□□　□□□□□│□□□□
□□□□□□□／□□　□□│□□□□□□□　□□□□□／□□□□
□□□□□□／□□□　□□│□□□□□□□　□□□□│□□□□□
□□□□□／□□□□　□□│□□□□□□□　□□□□／□□□□□
□□□□／□□□□□　□□│□□□□□□□　□□□│□□□□□□
□□□／□□□□□□　□□│□□□□□□□　□□□／□□□□□□
□□Ｓ□□□□□□□　□□Ｓ□□□□□□□　□□Ｓ□□□□□□□
□□□□□□□□□□　□□□□□□□□□□　□□□□□□□□□□

Objective:
In the case of pattern ③, adjust the negative cost to prioritize it, aiming for a more straight path.
Zigzag paths are smoothed into a shape closer to a straight line.
*/

namespace tmc_astar_lib {

const int32_t kDirectionNum = static_cast<int32_t>(NodeDirection::DIR_Max);

// Correction value definition
// Designed under the assumption that no turn exceeds 90° in a single step
// Adjacent: Parallel to the X-axis/Y-axis
// Diagonal: Along the diagonal direction
static constexpr int32_t kCostAdjAdjDiag = -12;   // Adjacent → Adjacent → Diagonal ――／
static constexpr int32_t kCostAdjDiagAdj = -4;    // Adjacent → Diagonal → Adjacent ―／―
static constexpr int32_t kCostAdjDiagDiag = -2;   // Adjacent → Diagonal → Diagonal ―／／
static constexpr int32_t kCostDiagDiagAdj = -11;  // Diagonal → Diagonal → Adjacent ／／―
static constexpr int32_t kCostDiagAdjDiag = -2;   // Diagonal → Adjacent → Diagonal ／―／
static constexpr int32_t kCostDiagAdjAdj = -4;    // Diagonal → Adjacent → Adjacent ／――

class DirectionCostCorrector : public ICostCorrector {
 public:
  /// Constructor
  DirectionCostCorrector();
  virtual ~DirectionCostCorrector();
  virtual void Setup(const SetupParams& params);
  virtual int32_t GetAdditionalCost(const GetAdditionalCostParams& params);
 private:
  // Outputs correction values based on the directions of the past three steps, storing all patterns in a table
  int32_t correction_table_[kDirectionNum][kDirectionNum][kDirectionNum];
};
}  // namespace tmc_astar_lib
#endif  // TMC_ASTAR_LIB_DIRECTION_COST_CORRECTOR_HPP_
