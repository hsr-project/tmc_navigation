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
#include <string>
#include <vector>
#include <gtest/gtest.h>

#include "../src/map_drawer.hpp"

namespace tmc_map_merger {

// Convert map to string
std::string ConvertMap(const std::vector<int8_t>& map) {
  std::string out;
  for (std::vector<int8_t>::const_iterator it = map.begin(); it != map.end(); ++it) {
    out.push_back((*it == kUnknown) ? '-' : *it + '0');
  }
  return out;
}

TEST(MapDrawerTest, Circle) {
  Map map;
  MapOperator<> op(map);
  drawer::MapPlotter<MapOperator<> > plotter(op);
  map.info.width = 8;
  map.info.height = 8;

  // Circle with radius 2
  op.Reset();
  drawer::Circle(plotter, 1, 3, 3, 2, true);
  std::vector<int8_t> exp;
  EXPECT_EQ(
      "--------"
      "--111---"
      "-11111--"
      "-11111--"
      "-11111--"
      "--111---"
      "--------"
      "--------",
      ConvertMap(map.data));

  // Circle with radius 3
  op.Reset();
  drawer::Circle(plotter, 2, 3, 3, 3, true);
  EXPECT_EQ(
      "--222---"
      "-22222--"
      "2222222-"
      "2222222-"
      "2222222-"
      "-22222--"
      "--222---"
      "--------",
      ConvertMap(map.data));

  // Circle with radius 3 (not filled)
  op.Reset();
  drawer::Circle(plotter, 2, 3, 3, 3, false);
  EXPECT_EQ(
      "--222---"
      "-2---2--"
      "2-----2-"
      "2-----2-"
      "2-----2-"
      "-2---2--"
      "--222---"
      "--------",
      ConvertMap(map.data));

  // Circle with radius 3 that overflows
  op.Reset();
  drawer::Circle(plotter, 3, 5, 6, 3, true);
  EXPECT_EQ(
      "--------"
      "--------"
      "--------"
      "----333-"
      "---33333"
      "--333333"
      "--333333"
      "--333333",
      ConvertMap(map.data));
}

}  // namespace tmc_map_merger

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
