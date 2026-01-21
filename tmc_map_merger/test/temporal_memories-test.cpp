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
#include <limits>

#include <gtest/gtest.h>

#include "../src/map.hpp"
#include "../src/temporal_memories.hpp"

namespace tmc_map_merger {

template<typename T>
class MemoryTypedTest : public testing::Test {};
TYPED_TEST_CASE_P(MemoryTypedTest);

/// @brief Verify that the timeout works when the timeout is enabled
TYPED_TEST_P(MemoryTypedTest, TimeoutEnabled) {
  // Setup
  TypeParam object;
  typename TypeParam::Option option;
  const double sec = 1.0;
  option.timeout = rclcpp::Duration::from_seconds(sec);
  option.default_data = kUnknown;
  object.Reset(rclcpp::Time(1), option);
  const int8_t obstacle = 100;

  // Excersise and verify
  object.Update(obstacle, rclcpp::Time(1), option);
  EXPECT_EQ(object.Get(rclcpp::Time(static_cast<int32_t>(sec), 0), option), obstacle);
  EXPECT_EQ(object.Get(rclcpp::Time(static_cast<int32_t>(sec), 1), option), kUnknown);
}

/// @brief Verify that there is no timeout when the timeout is disabled
TYPED_TEST_P(MemoryTypedTest, TimeoutDisabled) {
  // Setup
  TypeParam object;
  typename TypeParam::Option option;
  option.timeout = rclcpp::Duration::from_seconds(0.0);
  option.default_data = kUnknown;
  object.Reset(rclcpp::Time(1), option);
  const int8_t obstacle = 100;

  // Excersise and verify
  object.Update(obstacle, rclcpp::Time(1), option);
  EXPECT_EQ(object.Get(rclcpp::Time(1), option), obstacle);
  EXPECT_EQ(object.Get(rclcpp::Time(std::numeric_limits<int32_t>::max(), 999999999), option), obstacle);
}

// Add here when tests increase
REGISTER_TYPED_TEST_CASE_P(
    MemoryTypedTest,
    TimeoutEnabled,
    TimeoutDisabled);

// Add here when Memory classes increase
typedef testing::Types<
  SimpleMemory,
  SafetyMemory>
  MemoryTypes;

INSTANTIATE_TYPED_TEST_CASE_P(
    MemoryTypedTestInstance,
    MemoryTypedTest,
    MemoryTypes);

/// @brief Verify that the observed value is stored as is
TEST(SimpleMemoryTest, Priority) {
  // Setup
  SimpleMemory object;
  SimpleMemory::Option option;
  option.timeout = rclcpp::Duration::from_seconds(0.0);
  option.default_data = kUnknown;
  object.Reset(rclcpp::Time(1), option);
  const int8_t obstacle = 100;

  // Excersise and verify
  EXPECT_EQ(object.Get(rclcpp::Time(1), option), kUnknown);
  object.Update(kFree, rclcpp::Time(1), option);
  EXPECT_EQ(object.Get(rclcpp::Time(1), option), kFree);
  object.Update(kUnknown, rclcpp::Time(1), option);
  EXPECT_EQ(object.Get(rclcpp::Time(1), option), kFree);
  object.Update(obstacle, rclcpp::Time(1), option);
  EXPECT_EQ(object.Get(rclcpp::Time(1), option), obstacle);
  object.Update(obstacle - 1, rclcpp::Time(1), option);
  EXPECT_EQ(object.Get(rclcpp::Time(1), option), obstacle - 1);
  object.Reset(rclcpp::Time(1), option);
  EXPECT_EQ(object.Get(rclcpp::Time(1), option), kUnknown);
}

/// @brief Verify that if the observed value is greater than the stored value, it is stored, and if it is smaller, it is not stored
TEST(SafetyMemoryTest, Priority) {
  // Setup
  SafetyMemory object;
  SafetyMemory::Option option;
  option.timeout = rclcpp::Duration::from_seconds(0.0);
  option.default_data = kUnknown;
  object.Reset(rclcpp::Time(1), option);
  const int8_t obstacle = 100;

  // Excersise and verify
  EXPECT_EQ(object.Get(rclcpp::Time(1), option), kUnknown);
  object.Update(kFree, rclcpp::Time(1), option);
  EXPECT_EQ(object.Get(rclcpp::Time(1), option), kFree);
  object.Update(kUnknown, rclcpp::Time(1), option);
  EXPECT_EQ(object.Get(rclcpp::Time(1), option), kFree);
  object.Update(obstacle, rclcpp::Time(1), option);
  EXPECT_EQ(object.Get(rclcpp::Time(1), option), obstacle);
  object.Update(obstacle - 1, rclcpp::Time(1), option);
  EXPECT_EQ(object.Get(rclcpp::Time(1), option), obstacle);
  object.Reset(rclcpp::Time(1), option);
  EXPECT_EQ(object.Get(rclcpp::Time(1), option), kUnknown);
}
}  // namespace tmc_map_merger

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
