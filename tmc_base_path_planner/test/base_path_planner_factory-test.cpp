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
#include <memory>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tmc_base_path_planner/base_path_planner.hpp>
#include <tmc_base_path_planner/base_path_planner_factory.hpp>
#include "test_utils_ros.hpp"

namespace {
// Potential width of the static map [m]
constexpr double kStaticMapPotentialWidth = 3.0;
}   // anonymous namespace

namespace tmc_base_path_planner {
/// BasePathPlannerFactoryTest test fixture
class BasePathPlannerFactoryTest : public ::testing::Test {
 public:
  BasePathPlannerFactoryTest() {}

 protected:
  virtual void SetUp() {
    yaml_directory_ = ament_index_cpp::get_package_share_directory("tmc_base_path_planner") +
        "/test/parameter/";
    static_map_ = std::make_shared<CostMap>(Pose2d(), 0.05, 10, 10, std::vector<unsigned char>(100, 0));
    rclcpp::NodeOptions option;
    option.allow_undeclared_parameters();
    option.automatically_declare_parameters_from_overrides(true);
    test_node_ = std::make_shared<rclcpp::Node>("test_node", option);
  }

  std::shared_ptr<CostMap> static_map_;
  BasePathPlanner::Ptr base_path_planner_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::string yaml_directory_;
};


/// BasePathPlannerFactoryCreate test
/// Can be created when all parameters are specified
/// Whether it is generated according to the specified parameters is concealed and not visible from the outside, so it is not confirmed
TEST_F(BasePathPlannerFactoryTest, CreateWithAllParameterSpecified) {
  // setup
  LoadParameterFromYaml(test_node_, yaml_directory_,
      "base_path_planner_factory-all_parameter_specified_test.yaml");

  // exercise
  ASSERT_NO_THROW(base_path_planner_ = BasePathPlannerFactory::Create(
      test_node_, static_map_, kStaticMapPotentialWidth));

  // verify
  EXPECT_TRUE(base_path_planner_ != nullptr);
}


/// BasePathPlannerFactoryCreate test
/// Can be created even if no parameters are specified
/// Whether it is generated according to default values is concealed and not visible from the outside, so it is not confirmed
TEST_F(BasePathPlannerFactoryTest, CreateWithNoParameterSpecified) {
  // setup
  LoadParameterFromYaml(test_node_, yaml_directory_,
      "base_path_planner_factory-no_parameter_specified_test.yaml");

  // exercise
  ASSERT_NO_THROW(base_path_planner_ = BasePathPlannerFactory::Create(
      test_node_, static_map_, kStaticMapPotentialWidth));

  // verify
  EXPECT_TRUE(base_path_planner_ != nullptr);
}


/// BasePathPlannerFactoryCreate test
/// Throws an exception and cannot be created if an undefined planner is specified
TEST_F(BasePathPlannerFactoryTest, CreateWithUnknownPlannerTypeSpecified) {
  // setup
  LoadParameterFromYaml(test_node_, yaml_directory_,
      "base_path_planner_factory-unknown_planner_specified_test.yaml");

  // exercise and verify
  ASSERT_THROW(base_path_planner_ = BasePathPlannerFactory::Create(
      test_node_, static_map_, kStaticMapPotentialWidth), std::runtime_error);
}
}  // namespace tmc_base_path_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
