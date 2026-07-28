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
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <tmc_base_path_planner/astar_path_planner_factory.hpp>
#include <tmc_base_path_planner/param.hpp>
#include "test_utils_ros.hpp"

namespace {
// Potential width of static map [m]
constexpr double kStaticMapPotentialWidth = 3.0;
}   // anonymous namespace

namespace tmc_base_path_planner {
/// Test fixture for AstarPathPlannerFactory
class AstarPathPlannerFactoryTest : public ::testing::Test {
 public:
  AstarPathPlannerFactoryTest() {}

 protected:
  virtual void SetUp() {
    Pose2d static_map_origin(0.0, 0.0, 0.0);
    std::vector<uint8_t> static_map_data(1000 * 1000, 1);
    static_map_ = std::make_shared<CostMap>(CostMap(static_map_origin, 0.05, 1000, 1000, static_map_data));
    rclcpp::NodeOptions option;
    option.allow_undeclared_parameters();
    option.automatically_declare_parameters_from_overrides(true);
    test_node_ = std::make_shared<rclcpp::Node>("test_node", option);

    const std::string yaml_directory =
        ament_index_cpp::get_package_share_directory("tmc_base_path_planner") + "/test/parameter/";
    LoadParameterFromYaml(test_node_, yaml_directory, "astar_path_planner_factory-test.yaml");
  }

  AstarPathPlanner::Ptr planner_;
  CostMapPtr static_map_;
  std::shared_ptr<rclcpp::Node> test_node_;
};

/// AstarPathPlannerFactoryCreate test
/// Able to create when all parameters are specified
/// Whether it is generated according to the specified parameters is concealed and cannot be confirmed externally
TEST_F(AstarPathPlannerFactoryTest, CreateWithAllParameterSpecified) {
  // setup
  std::map<std::string, rclcpp::Parameter> params;
  ASSERT_TRUE(GetGroupParam(test_node_, "all_parameter_specified_test", params));

  // exercise
  ASSERT_NO_THROW(planner_ = AstarPathPlannerFactory::Create(params, static_map_, kStaticMapPotentialWidth));

  // verify
  EXPECT_TRUE(planner_ != nullptr);
}


/// AstarPathPlannerFactoryCreate test
/// Able to create even if no parameters are specified
/// Whether it is generated according to default values is concealed and cannot be confirmed externally
TEST_F(AstarPathPlannerFactoryTest, CreateWithNoParameterSpecified) {
  // setup
  std::map<std::string, rclcpp::Parameter> params;
  ASSERT_TRUE(GetGroupParam(test_node_, "no_parameter_specified_test", params));
  // exercise
  ASSERT_NO_THROW(planner_ = AstarPathPlannerFactory::Create(params, static_map_, kStaticMapPotentialWidth));

  // verify
  EXPECT_TRUE(planner_ != nullptr);
}

}  // namespace tmc_base_path_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
