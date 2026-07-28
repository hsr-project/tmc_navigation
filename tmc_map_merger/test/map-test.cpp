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
#include <string>

#include <gtest/gtest.h>

#include "../src/map.hpp"

namespace {
const double kEpsilon = 1e-9;  // Precision that cannot be achieved with float
}  // anonymous namespace

namespace tmc_map_merger {

/// Test for GetYawFromQuaternion function
TEST(MapUtilFunctionTest, GetYawFromQuaternion) {
  geometry_msgs::msg::Quaternion q;
  // Test with 0
  q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;
  EXPECT_NEAR(::GetYawFromQuaternion(q), 0.0, kEpsilon);

  // Test with PI
  const double PI = 3.1415926535897927;
  q.x = 0.0; q.y = 0.0; q.z = 1.0; q.w = 0.0;
  EXPECT_NEAR(::GetYawFromQuaternion(q), PI, kEpsilon);

  // Test that it falls between -PI and PI
  // (TODO) nishino kEplisonよりepsのほうが精度が高いので期待値の比較が微妙
  const double eps = std::numeric_limits<double>::epsilon();
  q.x = 0.0; q.y = 0.0; q.z = 1.0 - eps; q.w = eps;
  EXPECT_NEAR(::GetYawFromQuaternion(q), PI - eps, kEpsilon);
  q.x = 0.0; q.y = 0.0; q.z = -1.0 + eps; q.w = eps;
  EXPECT_NEAR(::GetYawFromQuaternion(q), -PI + eps, kEpsilon);
}

/// Test for round function
TEST(MapUtilFunctionTest, round) {
  EXPECT_NEAR(-2.0, ::round(-2.4999), kEpsilon);
  EXPECT_NEAR(-3.0, ::round(-2.5), kEpsilon);
  EXPECT_NEAR(2.0, ::round(2.4999), kEpsilon);
  EXPECT_NEAR(3.0, ::round(2.5), kEpsilon);
}

/// Test for SimpleMapOperator(Map/BasicAdapter)
TEST(MapOperatorTest, SimpleMapOperator) {
  Map map;
  map.info.width = 10;
  map.info.height = 10;
  SimpleMapOperator op(map);

  // Test for Reset()
  // Size and contents are reset
  op.Reset();
  ASSERT_EQ(100, map.data.size());
  for (size_t i = 0; i < 100; ++i) {
    EXPECT_EQ(kUnknown, map.data[i]);
  }
  // Modify the value, reset again, and verify
  for (size_t i = 0; i < 100; ++i) {
    map.data[i] = 10;
  }
  op.Reset();
  ASSERT_EQ(100, map.data.size());
  for (size_t i = 0; i < 100; ++i) {
    EXPECT_EQ(kUnknown, map.data[i]);
  }

  // Test for Get()
  // 3 can be retrieved
  map.data[20] = 3;
  EXPECT_EQ(3, op.Get(20));

  // Test for Update()
  // Updated with 20
  op.Update(50, 20);
  EXPECT_EQ(20, map.data[50]);

  // Test for Reset(index)
  // Reset with kUnknown
  op.Reset(50);
  EXPECT_EQ(kUnknown, map.data[50]);
}

/// Test for Map/UpdateIfGreaterMapAdapter
TEST(MapOperatorTest, UpdateIfGreaterMapAdapter) {
  Map map;
  map.info.width = 10;
  map.info.height = 10;
  MapOperator<UpdateIfGreaterMapAdapter> op(map);
  // Test for Reset()
  op.Reset();
  ASSERT_EQ(100, map.data.size());
  for (size_t i = 0; i < 100; ++i) {
    EXPECT_EQ(kUnknown, map.data[i]);
  }
  // Modify the value, reset again, and verify
  for (size_t i = 0; i < 100; ++i) {
    map.data[i] = 10;
  }
  op.Reset();
  ASSERT_EQ(100, map.data.size());
  for (size_t i = 0; i < 100; ++i) {
    EXPECT_EQ(kUnknown, map.data[i]);
  }

  // Test for Get()
  map.data[20] = 3;
  EXPECT_EQ(3, op.Get(20));

  // Test for Update()
  op.Update(50, 20);
  EXPECT_EQ(20, map.data[50]);
  op.Update(50, 30);
  EXPECT_EQ(30, map.data[50]);
  op.Update(50, 20);
  EXPECT_EQ(30, map.data[50]);  // Not updated

  // Test for Reset(index)
  op.Reset(50);
  EXPECT_EQ(kUnknown, map.data[50]);
}

struct X {
  X() : value(1000) {}  // Reset value is 1000
  explicit X(int v) : value(v) {}
  int value;
};

/// Test for AnyMap/BasicAdapter
TEST(MapOperatorTest, AnyMapBasicAdapter) {
  MapInfo info;
  info.width = 10;
  info.height = 10;
  AnyMap<X> map(info);
  MapOperator<BasicAdapter<X>, AnyMap<X> > op(map);

  // Test for Reset()
  // Size and contents are reset
  op.Reset();
  ASSERT_EQ(100, map.data.size());
  for (size_t i = 0; i < 100; ++i) {
    EXPECT_EQ(1000, map.data[i].value);
  }
  // Modify the value, reset again, and verify
  for (size_t i = 0; i < 100; ++i) {
    map.data[i].value = 10;
  }
  op.Reset();
  ASSERT_EQ(100, map.data.size());
  for (size_t i = 0; i < 100; ++i) {
    EXPECT_EQ(1000, map.data[i].value);
  }

  // Test for Get()
  map.data[20].value = 3;
  EXPECT_EQ(3, op.Get(20).value);

  // Test for Update()
  op.Update(50, X(7));
  EXPECT_EQ(7, map.data[50].value);

  // Test for Reset(index)
  op.Reset(50);
  EXPECT_EQ(1000, map.data[50].value);
}

struct XAdapter {
  typedef int DataType;
  // Returns data multiplied by 2
  void Get(int& dst, const struct X& src) const {
    dst = src.value * 2;
  }
  // Returns data multiplied by 2 as a string
  void Get(std::string& dst, const struct X& src) const {
    char buf[100];
    snprintf(buf, sizeof(buf), "%d", src.value * 2);
    dst = std::string(buf);
  }
  // Stores data divided by 2
  void Update(struct X& dst, int src) const {
    dst.value = src / 2;
  }
  // Receives and stores as a string
  void Update(struct X& dst, const char* src) const {
    int s = atoi(src);
    dst.value = s / 2;
  }
  // Initial value is set to -10
  void Reset(struct X& dst) const {
    dst.value = -10;
  }
};

/// Test for AnyMap/Custom Adapter
TEST(MapOperatorTest, AnyMapMyAdapter) {
  MapInfo info;
  info.width = 10;
  info.height = 10;
  AnyMap<X> map(info);
  MapOperator<XAdapter, AnyMap<X> > op(map);
  // Test for Reset()
  op.Reset();
  // Size and contents are reset
  // Reset value is -10
  ASSERT_EQ(100, map.data.size());
  for (size_t i = 0; i < 100; ++i) {
    EXPECT_EQ(-10, map.data[i].value);
  }
  // Modify the value, reset again, and verify
  for (size_t i = 0; i < 100; ++i) {
    map.data[i].value = 10;
  }
  op.Reset();
  ASSERT_EQ(100, map.data.size());
  for (size_t i = 0; i < 100; ++i) {
    EXPECT_EQ(-10, map.data[i].value);
  }

  // Test for Get()
  // 30 can be retrieved as 60
  map.data[20].value = 30;
  EXPECT_EQ(60, op.Get(20));
  // Can also be retrieved as "60" if type is specified
  EXPECT_EQ("60", op.Get<std::string>(20));

  // Test for Update()
  // Storing 20 results in 10 being stored
  op.Update(50, 20);
  EXPECT_EQ(10, map.data[50].value);

  // Storing "40" results in 20 being stored
  op.Update(50, "40");
  EXPECT_EQ(20, map.data[50].value);

  // Test for Reset(index)
  op.Reset(50);
  EXPECT_EQ(-10, map.data[50].value);
}

}  // namespace tmc_map_merger

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
