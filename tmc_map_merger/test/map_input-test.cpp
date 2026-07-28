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
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "../src/map_input.hpp"
#include "../src/param.hpp"
#include "test_utils.hpp"

namespace tmc_map_merger {

/// @note The parameters of the tuple are, respectively, the ROS parameter namespace and the Subscriber class name
class MapInputFactoryTest
    : public testing::TestWithParam<std::pair<std::string, std::string> > {
 protected:
  virtual void SetUp() {
    test_node_ = CreateParameterNode("map_input-test.yaml");
  }
  std::shared_ptr<rclcpp::Node> test_node_;
};

INSTANTIATE_TEST_CASE_P(
    MapInputFactoryTestSuccess,
    MapInputFactoryTest,
    testing::Values(std::make_pair("base_scan", typeid(Subscriber<sensor_msgs::msg::LaserScan>).name()),
                    std::make_pair("head_rgbd_sensor", typeid(Subscriber<sensor_msgs::msg::PointCloud2>).name()),
                    std::make_pair("obstacle_grid_map", typeid(Subscriber<nav_msgs::msg::OccupancyGrid>).name())));

TEST_P(MapInputFactoryTest, Success) {
  // Setup
  std::map<std::string, rclcpp::Parameter> params;
  ASSERT_NO_THROW(GetRequiredGroupParam(test_node_, std::string("inputs.") + GetParam().first, params));
  MapMerger::Ptr merger;
  ASSERT_NO_THROW(merger = MapMergerFactory::Create(params));
  MapInput::Ptr map_input;
  std::string input_name = "base_scan";

  // Excersise
  ASSERT_NO_THROW(map_input = MapInputFactory::Create(input_name, params, test_node_, merger));

  // Verify
  ASSERT_TRUE(map_input);
  EXPECT_EQ(GetParam().second, typeid(*map_input).name())
      << GetParam().second << " vs " << typeid(*map_input).name();
  EXPECT_TRUE(map_input->GetMapMerger());
}

TEST(MapInputFactoryTest, Failure) {
  // Setup
  std::shared_ptr<rclcpp::Node> test_node = CreateParameterNode("map_input-test.yaml");
  std::map<std::string, rclcpp::Parameter> params;
  ASSERT_NO_THROW(GetRequiredGroupParam(test_node, std::string("inputs.xxx"), params));
  MapMerger::Ptr merger;
  ASSERT_NO_THROW(merger = MapMergerFactory::Create(params));
  MapInput::Ptr map_input;
  std::string input_name = "base_scan";

  // Excersise
  ASSERT_ANY_THROW(map_input = MapInputFactory::Create(input_name, params, test_node, merger));

  // Verify
  ASSERT_FALSE(map_input);
}
}  // namespace tmc_map_merger

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
