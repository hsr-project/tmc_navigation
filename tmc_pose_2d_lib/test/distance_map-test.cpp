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
#include <algorithm>
#include <string>
#include <vector>
#include <gtest/gtest.h>
#include "tmc_pose_2d_lib/distance_map.hpp"

namespace tmc_pose_2d_lib {


TEST(DistanceMapTest, DispanceMapClassTest) {
  // Bottom left is map(1,2)
  // 0     0 0
  // 0   255 0
  // 50  100 0
  // 0     0 0

  // Number of data doesn't match
  EXPECT_THROW(DistanceMap invalid_map(
    Pose2d(1.0, 2.0, M_PI/2.0),
    0.1, 3, 3,
    std::vector<unsigned char>{
    0, 50, 0, 0,
    0, 100, 255, 0,
    1, 0, 0, 0}),
    std::runtime_error);

  // Number of data matches
  DistanceMap map(
    Pose2d(1.0, 2.0, M_PI/2.0),
    0.1, 4, 3,
    std::vector<unsigned char>{
    0, 50, 0, 0,
    0, 100, 255, 0,
    1, 0, 0, 0});
  map.SetPotentialWidth(0.2);

  // Inside the map
  ASSERT_TRUE(map.InMap(Point2d(0.8, 2.2)));

  // Outside the map
  ASSERT_FALSE(map.InMap(Point2d(0.0, 0.0)));
  ASSERT_FALSE(map.InMap(Point2d(0.8, 1.8)));
  ASSERT_FALSE(map.InMap(Point2d(0.8, 2.5)));
  ASSERT_FALSE(map.InMap(Point2d(0.6, 2.2)));
  ASSERT_FALSE(map.InMap(Point2d(1.2, 2.2)));


  // Test of distance
  const double very_small_val = 0.000000001;
  DistanceMap::CellType type;
  double distance;
  printf("-------------------------------\n");
  // HasDistance area
  map.CheckTypeAndDistance(Point2d(0.94, 2.14), type, distance);
  ASSERT_NEAR(distance, (255.0 - 50.0)/255.0*0.2, very_small_val);
  ASSERT_TRUE(type == DistanceMap::CellType::kHasDistance);

  map.CheckTypeAndDistance(Point2d(0.81, 2.11), type, distance);
  ASSERT_NEAR(distance, (255.0- 100.0)/255.0*0.2, very_small_val);
  ASSERT_TRUE(type == DistanceMap::CellType::kHasDistance);

  map.CheckTypeAndDistance(Point2d(0.85, 2.21), type, distance);
  ASSERT_NEAR(distance, (255.0- 255.0)/255.0*0.2, very_small_val);
  ASSERT_TRUE(type == DistanceMap::CellType::kHasDistance);

  // UnExplored area
  map.CheckTypeAndDistance(Point2d(0.73, 2.31), type, distance);
  ASSERT_TRUE(type == DistanceMap::CellType::kUnExplored);

  // Free area
  map.CheckTypeAndDistance(Point2d(0.75, 2.05), type, distance);
  ASSERT_TRUE(type == DistanceMap::CellType::kFree);

  // Outside area
  map.CheckTypeAndDistance(Point2d(0.0, 0.0), type, distance);
  ASSERT_TRUE(type == DistanceMap::CellType::kOutside);

  map.CheckTypeAndDistance(Point2d(1.01, 2.3), type, distance);
  ASSERT_TRUE(type == DistanceMap::CellType::kOutside);
}

// Test that potential can expand within a specified distance range from the wall
TEST(DistanceMapTest, DispanceMapInflateMapTest) {
  DistanceMap map(
    Pose2d(0.0, 0.0, 0.0),
    0.1, 9, 9,
    std::vector<unsigned char>{
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0});
  // Map with a wall in the center of the map
  const int32_t wall_u = 4;
  const int32_t wall_v = 4;
  map.SetValueAt(wall_u, wall_v, 255);
  // Expand
  const double inflate_width = 0.41;
  map.InflateMap(inflate_width);
  for (int32_t u = 0; u < map.width(); ++u) {
    for (int32_t v = 0; v < map.height(); ++v) {
      const int32_t current = u + v * map.width();
      const double distance_to_wall = sqrt(pow(map.resolution() * (wall_u - u), 2) +
                                           pow(map.resolution() * (wall_v - v), 2));
      if (distance_to_wall < inflate_width) {
        // Within the expanded range, the potential relates to the distance from the wall
        const unsigned char expect_value = 255 - static_cast<unsigned char>(254 * (distance_to_wall / inflate_width));
        ASSERT_EQ(expect_value, map.data().at(current));
      } else {
        // Outside the range has not changed
        ASSERT_EQ(0, map.data().at(current));
      }
    }
  }
}

// When expanding potential, the grid between two walls is affected by the closer wall
TEST(DistanceMapTest, DispanceMapInflateMapByNearestWallTest) {
  DistanceMap map(
    Pose2d(0.0, 0.0, 0.0),
    0.1, 9, 9,
    std::vector<unsigned char>{
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0});
  // Map with walls in the center and origin of the map
  const int32_t wall1_u = 4;
  const int32_t wall1_v = 4;
  const int32_t wall2_u = 0;
  const int32_t wall2_v = 0;
  map.SetValueAt(wall1_u, wall1_v, 255);
  map.SetValueAt(wall2_u, wall2_v, 255);
  // Expand
  const double inflate_width = 0.41;
  map.InflateMap(inflate_width);
  for (int32_t u = 0; u < map.width(); ++u) {
    for (int32_t v = 0; v < map.height(); ++v) {
      const int32_t current = u + v * map.width();
      const double distance_to_wall1 = sqrt(pow(map.resolution() * (wall1_u - u), 2) +
                                            pow(map.resolution() * (wall1_v - v), 2));
      const double distance_to_wall2 = sqrt(pow(map.resolution() * (wall2_u - u), 2) +
                                            pow(map.resolution() * (wall2_v - v), 2));
      if (distance_to_wall1 < inflate_width || distance_to_wall2 < inflate_width) {
        // Potential according to the distance from the closer wall
        const double distance_to_wall = std::min(distance_to_wall1, distance_to_wall2);
        const unsigned char expect_value = 255 - static_cast<unsigned char>(254 * (distance_to_wall / inflate_width));
        ASSERT_EQ(expect_value, map.data().at(current));
      } else {
        // Outside the range has not changed
        ASSERT_EQ(0, map.data().at(current));
      }
    }
  }
}
}  // end namespace tmc_pose_2d_lib

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
