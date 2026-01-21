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

#include <tmc_base_path_follower/base_path_follower.hpp>
#include <tmc_base_path_follower/base_path_follower_factory.hpp>
#include "test_utils.hpp"

namespace tmc_base_path_follower {

/// BasePathFollowerFactoryTest test fixture
class BasePathFollowerFactoryTest : public ::testing::Test {
 public:
  BasePathFollowerFactoryTest() {}

 protected:
  void SetUp() {
    yaml_directory_ = ament_index_cpp::get_package_share_directory("tmc_base_path_follower") +
        "/test/parameter/";
    rclcpp::NodeOptions option;
    option.allow_undeclared_parameters();
    option.automatically_declare_parameters_from_overrides(true);
    test_node_ = std::make_shared<rclcpp::Node>("base_path_follower", option);
  }

  BasePathFollower::Ptr base_path_follower_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::string yaml_directory_;
};


/// BasePathFollowerFactoryCreate test
/// Can create when all parameters for the omni movement model are specified
/// Do not verify if it is generated according to the specified parameters, as it is hidden and not visible from the outside
TEST_F(BasePathFollowerFactoryTest, CreateOmniWithAllParameter) {
  // setup
  LoadParameterFromYaml(test_node_, yaml_directory_,
      "base_path_follower_factory-test_omni_model_test.yaml");

  // exercise
  ASSERT_NO_THROW(base_path_follower_ = CreateBasePathFollower(test_node_));

  // verify
  EXPECT_TRUE(base_path_follower_ != nullptr);
}


/// BasePathFollowerFactoryCreate test
/// Can create when all parameters for the diff_drive movement model are specified
/// Do not verify if it is generated according to the specified parameters, as it is hidden and not visible from the outside
TEST_F(BasePathFollowerFactoryTest, CreateDiffDriveWithAllParameter) {
  // setup
  LoadParameterFromYaml(test_node_, yaml_directory_,
      "base_path_follower_factory-test_diff_drive_model_test.yaml");

  // exercise
  ASSERT_NO_THROW(base_path_follower_ = CreateBasePathFollower(test_node_));

  // verify
  EXPECT_TRUE(base_path_follower_ != nullptr);
}

/// BasePathFollowerFactoryCreate test
/// Throws an exception and cannot create when an undefined movement model is specified
TEST_F(BasePathFollowerFactoryTest, CreateWithUnknownMoveModel) {
  // setup
    LoadParameterFromYaml(test_node_, yaml_directory_,
      "base_path_follower_factory-test_unknown_move_model_test.yaml");

  // exercise and verify
  ASSERT_THROW(base_path_follower_ = CreateBasePathFollower(test_node_),
               std::runtime_error);
}

/// BasePathFollowerFactoryCreate test
/// Can create even if no parameters are specified
/// Do not verify if it is generated according to the default values, as it is hidden and not visible from the outside
TEST_F(BasePathFollowerFactoryTest, CreateWithNoParameter) {
  // setup
  LoadParameterFromYaml(test_node_, yaml_directory_,
      "base_path_follower_factory-test_no_parameter_test.yaml");

  // exercise
  ASSERT_NO_THROW(base_path_follower_ = CreateBasePathFollower(test_node_));

  // verify
  EXPECT_TRUE(base_path_follower_ != nullptr);
}
}  // namespace tmc_base_path_follower

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
