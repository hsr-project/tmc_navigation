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
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "../src/param.hpp"
#include "test_utils.hpp"
namespace tmc_map_merger {

// TODO(fukukazu_kawata_zb) 型パラメータテストをすることを検討

TEST(ParamTest, Greater) {
  // Setup
  Greater<int> object = Greater<int>(0);

  // Excersise and verify
  EXPECT_TRUE(object.validate(1));
  EXPECT_FALSE(object.validate(0));
  EXPECT_EQ("should be greater than 0", object.message());
}

TEST(ParamTest, NotLess) {
  // Setup
  NotLess<int> object = NotLess<int>(0);

  // Excersise and verify
  EXPECT_TRUE(object.validate(0));
  EXPECT_FALSE(object.validate(-1));
  EXPECT_EQ("should not be less than 0", object.message());
}

TEST(ParamTest, InRange) {
  // Setup
  InRange<int> object = InRange<int>(0, 2);

  // Excersise and verify
  EXPECT_FALSE(object.validate(-1));
  EXPECT_TRUE(object.validate(0));
  EXPECT_TRUE(object.validate(1));
  EXPECT_TRUE(object.validate(2));
  EXPECT_FALSE(object.validate(3));
  EXPECT_EQ("should be within the range between 0 and 2", object.message());
}

TEST(ParamTest, OneOfInvalidReference) {
  // Setup
  std::vector<int> ref;
  ref.push_back(0);
  std::shared_ptr<OneOf<int> > object_ptr;

  // Excersise and vefiry
  EXPECT_ANY_THROW(object_ptr.reset(new OneOf<int>(ref)));
}

TEST(ParamTest, OneOfValidMinLengthReference) {
  // Setup
  std::vector<int> ref;
  ref.push_back(0);
  ref.push_back(1);
  std::shared_ptr<OneOf<int> > object_ptr;

  // Excersise
  ASSERT_NO_THROW(object_ptr.reset(new OneOf<int>(ref)));

  // Verify
  EXPECT_TRUE(object_ptr->validate(0));
  EXPECT_TRUE(object_ptr->validate(1));
  EXPECT_FALSE(object_ptr->validate(2));
  EXPECT_EQ("should be one of 0 or 1", object_ptr->message());
}

TEST(ParamTest, OneOfValidLengthReference) {
  // Setup
  std::vector<int> ref;
  ref.push_back(0);
  ref.push_back(1);
  ref.push_back(2);
  std::shared_ptr<OneOf<int> > object_ptr;

  // Excersise
  ASSERT_NO_THROW(object_ptr.reset(new OneOf<int>(ref)));

  // Verify
  EXPECT_TRUE(object_ptr->validate(0));
  EXPECT_TRUE(object_ptr->validate(1));
  EXPECT_TRUE(object_ptr->validate(2));
  EXPECT_FALSE(object_ptr->validate(3));
  EXPECT_EQ("should be one of 0, 1 or 2", object_ptr->message());
}

class RosParamTest : public testing::Test {
 protected:
  virtual void SetUp() {
    test_node_ = CreateParameterNode("param-test.yaml");
  }
  std::shared_ptr<rclcpp::Node> test_node_;
};


TEST_F(RosParamTest, RequiredParamRos) {
  // Setup
  Greater<int> greater_than_zero(0);
  int valid_with_checker = std::numeric_limits<int>::max();
  int invalid_with_checker = std::numeric_limits<int>::max();
  int valid_without_checker = std::numeric_limits<int>::max();
  int invalid_without_checker = std::numeric_limits<int>::max();

  // Excersise
  ASSERT_NO_THROW(GetRequiredParam(test_node_, "root.one", valid_with_checker, greater_than_zero));
  ASSERT_ANY_THROW(GetRequiredParam(test_node_, "root.zero", invalid_with_checker, greater_than_zero));
  ASSERT_NO_THROW(GetRequiredParam(test_node_, "root.one", valid_without_checker));
  ASSERT_ANY_THROW(GetRequiredParam(test_node_, "not_exists", invalid_without_checker));

  // Verify
  EXPECT_EQ(1, valid_with_checker);
  EXPECT_EQ(std::numeric_limits<int>::max(), invalid_with_checker);
  EXPECT_EQ(1, valid_without_checker);
  EXPECT_EQ(std::numeric_limits<int>::max(), invalid_without_checker);
}

TEST_F(RosParamTest, RequiredGroupParam) {
  // Setup
  Greater<int> greater_than_zero(0);
  std::map<std::string, rclcpp::Parameter> group;
  ASSERT_NO_THROW(GetRequiredGroupParam(test_node_, "root", group));

  int valid_with_checker = std::numeric_limits<int>::max();
  int invalid_with_checker = std::numeric_limits<int>::max();
  int valid_without_checker = std::numeric_limits<int>::max();
  int invalid_without_checker = std::numeric_limits<int>::max();

  // Excersise
  ASSERT_NO_THROW(GetRequiredParam(group, "one", valid_with_checker, greater_than_zero));
  ASSERT_ANY_THROW(GetRequiredParam(group, "zero", invalid_with_checker, greater_than_zero));
  ASSERT_NO_THROW(GetRequiredParam(group, "one", valid_without_checker));
  ASSERT_ANY_THROW(GetRequiredParam(group, "not_exists", invalid_without_checker));

  // Verify
  EXPECT_EQ(1, valid_with_checker);
  EXPECT_EQ(std::numeric_limits<int>::max(), invalid_with_checker);
  EXPECT_EQ(1, valid_without_checker);
  EXPECT_EQ(std::numeric_limits<int>::max(), invalid_without_checker);
}

TEST_F(RosParamTest, OptionalParamRos) {
  // Setup
  Greater<int> greater_than_zero(0);
  int valid_with_checker = std::numeric_limits<int>::max();
  int invalid_with_checker = std::numeric_limits<int>::max();
  int valid_without_checker = std::numeric_limits<int>::max();
  int invalid_without_checker = std::numeric_limits<int>::max();
  int contradictory_default_value = std::numeric_limits<int>::max();
  const int valid_default_value = 2;
  const int invalid_default_value = 0;

  // Excersise
  ASSERT_NO_THROW(GetOptionalParam(test_node_, "root.one", valid_with_checker, valid_default_value, greater_than_zero));
  ASSERT_NO_THROW(
      GetOptionalParam(test_node_, "root.zero", invalid_with_checker, valid_default_value, greater_than_zero));
  ASSERT_NO_THROW(GetOptionalParam(test_node_, "root.one", valid_without_checker, valid_default_value));
  ASSERT_NO_THROW(GetOptionalParam(test_node_, "not_exists", invalid_without_checker, valid_default_value));
  ASSERT_ANY_THROW(
      GetOptionalParam(test_node_, "root.zero", contradictory_default_value, invalid_default_value, greater_than_zero));

  // Verify
  EXPECT_EQ(1, valid_with_checker);
  EXPECT_EQ(valid_default_value, invalid_with_checker);
  EXPECT_EQ(1, valid_without_checker);
  EXPECT_EQ(valid_default_value, invalid_without_checker);
  EXPECT_EQ(std::numeric_limits<int>::max(), contradictory_default_value);
}

TEST_F(RosParamTest, OptionalGroupParam) {
  // Setup
  Greater<int> greater_than_zero(0);
  std::map<std::string, rclcpp::Parameter> group;
  ASSERT_NO_THROW(GetRequiredGroupParam(test_node_, "root", group));
  int valid_with_checker = std::numeric_limits<int>::max();
  int invalid_with_checker = std::numeric_limits<int>::max();
  int valid_without_checker = std::numeric_limits<int>::max();
  int invalid_without_checker = std::numeric_limits<int>::max();
  int contradictory_default_value = std::numeric_limits<int>::max();
  const int valid_default_value = 2;
  const int invalid_default_value = 0;

  // Excersise
  ASSERT_NO_THROW(GetOptionalParam(group, "one", valid_with_checker, valid_default_value, greater_than_zero));
  ASSERT_NO_THROW(GetOptionalParam(group, "zero", invalid_with_checker, valid_default_value, greater_than_zero));
  ASSERT_NO_THROW(GetOptionalParam(group, "one", valid_without_checker, valid_default_value));
  ASSERT_NO_THROW(GetOptionalParam(group, "not_exists", invalid_without_checker, valid_default_value));
  ASSERT_ANY_THROW(
      GetOptionalParam(group, "zero", contradictory_default_value, invalid_default_value, greater_than_zero));

  // Verify
  EXPECT_EQ(1, valid_with_checker);
  EXPECT_EQ(valid_default_value, invalid_with_checker);
  EXPECT_EQ(1, valid_without_checker);
  EXPECT_EQ(valid_default_value, invalid_without_checker);
  EXPECT_EQ(std::numeric_limits<int>::max(), contradictory_default_value);
}
}  // end of namespace tmc_map_merger

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
