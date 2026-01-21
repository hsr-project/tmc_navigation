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
#include <cmath>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "../src/param.hpp"
#include "../src/point_cloud_filter.hpp"
#include "test_utils.hpp"

namespace tmc_map_merger {

typedef std::pair<std::string, std::string> TestPair;
/// @note The parameters of the pair are the ROS parameter namespace and the Subscriber class name respectively
class PointCloudFilterFactoryTest
    : public testing::TestWithParam<TestPair> {

 protected:
  virtual void SetUp() {
    test_node_ = CreateParameterNode("point_cloud_filter-test.yaml");
  }
  std::shared_ptr<rclcpp::Node> test_node_;
};

INSTANTIATE_TEST_CASE_P(
    PointCloudFilterFactoryTestSuccess,
    PointCloudFilterFactoryTest,
    testing::Values(TestPair("voxel_grid", typeid(PointCloudVoxelGridFilter).name()),
                    TestPair("transform", typeid(PointCloudTransformFilter).name()),
                    TestPair("trimming", typeid(PointCloudTrimmingFilter).name()),
                    TestPair("noise", typeid(PointCloudNoiseFilter).name())));

TEST_P(PointCloudFilterFactoryTest, FactorySuccess) {
  // Setup
  std::map<std::string, rclcpp::Parameter> params;
  GetRequiredGroupParam(test_node_, std::string("point_cloud_filters_") + GetParam().first, params);
  PointCloudFilter::Ptr object_ptr;

  // Excersise
  ASSERT_NO_THROW(object_ptr = PointCloudFilterFactory::Create(params));

  // Verify
  ASSERT_TRUE(object_ptr);
  EXPECT_EQ(GetParam().second, typeid(*object_ptr).name());
}

TEST(PointCloudFilterFactoryTest, FactoryFailure) {
  // Setup
  std::shared_ptr<rclcpp::Node> node = CreateParameterNode("point_cloud_filter-test.yaml");
  std::map<std::string, rclcpp::Parameter> params;
  GetRequiredGroupParam(node, std::string("point_cloud_filters_xxx"), params);
  PointCloudFilter::Ptr object_ptr;

  // Excersise
  ASSERT_ANY_THROW(object_ptr = PointCloudFilterFactory::Create(params));

  // Verify
  ASSERT_FALSE(object_ptr);
}

template<typename T>
class PointCloudFilterTypedTest : public testing::Test {};
TYPED_TEST_CASE_P(PointCloudFilterTypedTest);

/// @brief Ensure that the output is empty when the input is empty
TYPED_TEST_P(PointCloudFilterTypedTest, Empty) {
  // Setup
  PointCloud::Ptr cloud_in(new PointCloud());
  PointCloud::Ptr cloud_out(new PointCloud());
  std::vector<PointCloudFilter::Ptr> object_ptrs;
  ASSERT_NO_THROW(object_ptrs.push_back(PointCloudFilter::Ptr(new TypeParam())));
  cloud_out->points.push_back(Point3f(1, 0, 0));
  EXPECT_EQ(1, cloud_out->size());

  // Excersise end verify
  for (size_t i = 0; i < object_ptrs.size(); ++i) {
    EXPECT_NO_THROW(object_ptrs[i]->Filter(cloud_in, Eigen::Affine3d::Identity(), cloud_out))
        << typeid(*(object_ptrs[i])).name();
    EXPECT_EQ(0, cloud_in->size()) << typeid(*(object_ptrs[i])).name();
    EXPECT_EQ(0, cloud_out->size()) << typeid(*(object_ptrs[i])).name();

    cloud_in.reset(new PointCloud());
    cloud_out.reset(new PointCloud());
  }
}

// Add here when tests increase
REGISTER_TYPED_TEST_CASE_P(
    PointCloudFilterTypedTest,
    Empty);

// Add here when types increase
typedef testing::Types<
  PointCloudVoxelGridFilter,
  PointCloudTrimmingFilter,
  PointCloudNoiseFilter,
  PointCloudTransformFilter>
  PointCloudFilterTypes;

INSTANTIATE_TYPED_TEST_CASE_P(
    PointCloudFilterTypedTestInstance,
    PointCloudFilterTypedTest,
    PointCloudFilterTypes);

/// @brief Ensure that concentrated points are reduced to one point
/// @note Confirm that the position of the points after processing is within the voxel (do not worry about which point is representative)
TEST(PointCloudFilterTest, VoxelGrid) {
  // Setup
  PointCloud::Ptr cloud_in(new PointCloud());
  const int data_num = 10;
  for (int i = 0; i < data_num; ++i) {
    cloud_in->push_back(Point3f(0, 0, 0));
  }
  PointCloud::Ptr output_default(new PointCloud());
  PointCloud::Ptr output_specified(new PointCloud());
  PointCloudFilter::Ptr object_ptr_default(new PointCloudVoxelGridFilter());
  Eigen::Vector3d leaf_size;
  leaf_size << 0.05, 0.05, 0.05;
  PointCloudFilter::Ptr object_ptr_speficied(new PointCloudVoxelGridFilter(leaf_size));
  const float half_leaf_size = 0.025;

  // Excersise
  ASSERT_NO_THROW(object_ptr_default->Filter(cloud_in, Eigen::Affine3d::Identity(), output_default));
  ASSERT_NO_THROW(object_ptr_speficied->Filter(cloud_in, Eigen::Affine3d::Identity(), output_specified));

  // Verify
  EXPECT_EQ(data_num, cloud_in->size());
  ASSERT_EQ(1, output_default->size());
  EXPECT_GT(output_default->points[0].x, -half_leaf_size);
  EXPECT_GT(output_default->points[0].y, -half_leaf_size);
  EXPECT_GT(output_default->points[0].z, -half_leaf_size);
  EXPECT_LT(output_default->points[0].x, half_leaf_size);
  EXPECT_LT(output_default->points[0].y, half_leaf_size);
  EXPECT_LT(output_default->points[0].z, half_leaf_size);
  ASSERT_EQ(output_specified->size(), output_default->size());
  EXPECT_EQ(output_default->points[0].x, output_specified->points[0].x);
  EXPECT_EQ(output_default->points[0].y, output_specified->points[0].y);
  EXPECT_EQ(output_default->points[0].z, output_specified->points[0].z);
}

/// @brief Boundary value test for upper and lower limit filter
TEST(PointCloudFilterTest, Trimming) {
  // Setup
  PointCloud::Ptr min_cloud_in(new PointCloud());
  PointCloud::Ptr max_cloud_in(new PointCloud());
  const float min_threshold = 0.15;
  const float max_threshold = 1.2;
  const int data_num = 10;
  for (int i = 0; i < data_num; ++i) {
    if (i % 2) {
      min_cloud_in->push_back(Point3f(0, 0, min_threshold));
      max_cloud_in->push_back(Point3f(0, 0, max_threshold));
    } else {
      min_cloud_in->push_back(Point3f(0, 0, min_threshold - std::numeric_limits<float>::epsilon()));
      max_cloud_in->push_back(Point3f(0, 0, max_threshold + std::numeric_limits<float>::epsilon()));
    }
  }
  PointCloud::Ptr min_output_default(new PointCloud());
  PointCloud::Ptr max_output_default(new PointCloud());
  PointCloud::Ptr min_output_specified(new PointCloud());
  PointCloud::Ptr max_output_specified(new PointCloud());
  PointCloudFilter::Ptr object_ptr_default(new PointCloudTrimmingFilter());
  PointCloudFilter::Ptr object_ptr_speficied(new PointCloudTrimmingFilter("z", min_threshold, max_threshold));

  // Excersise
  ASSERT_NO_THROW(object_ptr_default->Filter(min_cloud_in, Eigen::Affine3d::Identity(), min_output_default));
  ASSERT_NO_THROW(object_ptr_default->Filter(max_cloud_in, Eigen::Affine3d::Identity(), max_output_default));
  ASSERT_NO_THROW(object_ptr_speficied->Filter(min_cloud_in, Eigen::Affine3d::Identity(), min_output_specified));
  ASSERT_NO_THROW(object_ptr_speficied->Filter(max_cloud_in, Eigen::Affine3d::Identity(), max_output_specified));

  // Verify
  ASSERT_EQ(data_num / 2, min_output_default->size());
  for (size_t i = 0; i < min_output_default->size(); ++i) {
    EXPECT_FLOAT_EQ(0, min_output_default->points[i].x) << "i = " << i;
    EXPECT_FLOAT_EQ(0, min_output_default->points[i].y) << "i = " << i;
    EXPECT_FLOAT_EQ(min_threshold, min_output_default->points[i].z) << "i = " << i;
  }
  ASSERT_EQ(data_num / 2, max_output_default->size());
  for (size_t i = 0; i < max_output_default->size(); ++i) {
    EXPECT_FLOAT_EQ(0, max_output_default->points[i].x) << "i = " << i;
    EXPECT_FLOAT_EQ(0, max_output_default->points[i].y) << "i = " << i;
    EXPECT_FLOAT_EQ(max_threshold, max_output_default->points[i].z) << "i = " << i;
  }
  ASSERT_EQ(min_output_default->size(), min_output_specified->size());
  for (size_t i = 0; i < max_output_default->size(); ++i) {
    EXPECT_FLOAT_EQ(min_output_default->points[i].x, min_output_specified->points[i].x) << "i = " << i;
    EXPECT_FLOAT_EQ(min_output_default->points[i].y, min_output_specified->points[i].y) << "i = " << i;
    EXPECT_FLOAT_EQ(min_output_default->points[i].z, min_output_specified->points[i].z) << "i = " << i;
    EXPECT_FLOAT_EQ(max_output_default->points[i].x, max_output_specified->points[i].x) << "i = " << i;
    EXPECT_FLOAT_EQ(max_output_default->points[i].y, max_output_specified->points[i].y) << "i = " << i;
    EXPECT_FLOAT_EQ(max_output_default->points[i].z, max_output_specified->points[i].z) << "i = " << i;
  }
}

/// @brief Boundary value test for noise filter
TEST(PointCloudFilterTest, Noise) {
  // Setup
  PointCloud::Ptr one_noise_cloud_in(new PointCloud());
  PointCloud::Ptr no_noise_cloud_in(new PointCloud());
  const int data_num = 10;
  const float radius = 0.1;
  const int min_neighbors = 1;
  for (int i = 0; i < data_num; ++i) {
    if (i < data_num - min_neighbors) {
      one_noise_cloud_in->push_back(Point3f(0, 0, 0));
      no_noise_cloud_in->push_back(Point3f(0, 0, 0));
    } else {
      one_noise_cloud_in->push_back(Point3f(0, 0, radius));
      no_noise_cloud_in->push_back(Point3f(0, 0, radius - std::numeric_limits<float>::epsilon()));
    }
  }
  PointCloud::Ptr one_noise_output_default(new PointCloud());
  PointCloud::Ptr no_noise_output_default(new PointCloud());
  PointCloud::Ptr one_noise_output_specified(new PointCloud());
  PointCloud::Ptr no_noise_output_specified(new PointCloud());
  PointCloudFilter::Ptr object_ptr_default(new PointCloudNoiseFilter());
  PointCloudFilter::Ptr object_ptr_specified(new PointCloudNoiseFilter());

  // Excersise
  ASSERT_NO_THROW(
      object_ptr_default->Filter(one_noise_cloud_in, Eigen::Affine3d::Identity(), one_noise_output_default));
  ASSERT_NO_THROW(
      object_ptr_default->Filter(no_noise_cloud_in, Eigen::Affine3d::Identity(), no_noise_output_default));
  ASSERT_NO_THROW(
      object_ptr_specified->Filter(one_noise_cloud_in, Eigen::Affine3d::Identity(), one_noise_output_specified));
  ASSERT_NO_THROW(
      object_ptr_specified->Filter(no_noise_cloud_in, Eigen::Affine3d::Identity(), no_noise_output_specified));

  // Verify
  ASSERT_EQ(data_num - min_neighbors, one_noise_output_default->size());
  for (size_t i = 0; i < one_noise_output_default->size(); ++i) {
    EXPECT_FLOAT_EQ(0, one_noise_output_default->points[i].x) << "i = " << i;
    EXPECT_FLOAT_EQ(0, one_noise_output_default->points[i].y) << "i = " << i;
    EXPECT_FLOAT_EQ(0, one_noise_output_default->points[i].z) << "i = " << i;
  }
  ASSERT_EQ(data_num, no_noise_output_default->size());
  for (size_t i = 0; i < no_noise_output_default->size(); ++i) {
    EXPECT_FLOAT_EQ(0, no_noise_output_default->points[i].x) << "i = " << i;
    EXPECT_FLOAT_EQ(0, no_noise_output_default->points[i].y) << "i = " << i;
    if (i <  no_noise_output_default->size() - min_neighbors) {
      EXPECT_FLOAT_EQ(0, no_noise_output_default->points[i].z) << "i = " << i;
    } else {
      EXPECT_FLOAT_EQ(
          radius - std::numeric_limits<float>::epsilon(), no_noise_output_default->points[i].z) << "i = " << i;
    }
  }
  ASSERT_EQ(one_noise_output_specified->size(), one_noise_output_default->size());
  for (size_t i = 0; i < one_noise_output_default->size(); ++i) {
    EXPECT_FLOAT_EQ(one_noise_output_specified->points[i].x, one_noise_output_default->points[i].x) << "i = " << i;
    EXPECT_FLOAT_EQ(one_noise_output_specified->points[i].y, one_noise_output_default->points[i].y) << "i = " << i;
    EXPECT_FLOAT_EQ(one_noise_output_specified->points[i].z, one_noise_output_default->points[i].z) << "i = " << i;
  }
  ASSERT_EQ(no_noise_output_specified->size(), no_noise_output_default->size());
  for (size_t i = 0; i < no_noise_output_default->size(); ++i) {
    EXPECT_FLOAT_EQ(no_noise_output_specified->points[i].x, no_noise_output_default->points[i].x) << "i = " << i;
    EXPECT_FLOAT_EQ(no_noise_output_specified->points[i].y, no_noise_output_default->points[i].y) << "i = " << i;
    EXPECT_FLOAT_EQ(no_noise_output_specified->points[i].z, no_noise_output_default->points[i].z) << "i = " << i;
  }
}

/// @brief Test for coordinate transformation filter
TEST(PointCloudFilterTest, Transform) {
  // Setup
  Eigen::Affine3d empty;
  empty.matrix() << 0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, 0;
  const Eigen::Affine3d identity(Eigen::Affine3d::Identity());
  const double half_sqrt2 = sqrt(2.0) / 2.0;
  Eigen::Affine3d transform;
  transform.matrix() << half_sqrt2, -half_sqrt2, 0, 0,
                        half_sqrt2,  half_sqrt2, 0, 0,
                                 0,           0, 0, 1,
                                 0,           0, 0, 1;
  PointCloudFilter::Ptr object_ptr(new PointCloudTransformFilter());
  PointCloud::Ptr cloud_in(new PointCloud());
  cloud_in->push_back(Point3f(1, 0, 0));
  PointCloud::Ptr empty_transform_output(new PointCloud());
  PointCloud::Ptr identity_transform_output(new PointCloud());
  PointCloud::Ptr transform_output(new PointCloud());

  // Excersise
  ASSERT_NO_THROW(object_ptr->Filter(cloud_in, empty, empty_transform_output));
  ASSERT_NO_THROW(object_ptr->Filter(cloud_in, identity, identity_transform_output));
  ASSERT_NO_THROW(object_ptr->Filter(cloud_in, transform, transform_output));

  // Verify
  EXPECT_FLOAT_EQ(0, empty_transform_output->points[0].x);
  EXPECT_FLOAT_EQ(0, empty_transform_output->points[0].y);
  EXPECT_FLOAT_EQ(0, empty_transform_output->points[0].z);
  EXPECT_FLOAT_EQ(1, identity_transform_output->points[0].x);
  EXPECT_FLOAT_EQ(0, identity_transform_output->points[0].y);
  EXPECT_FLOAT_EQ(0, identity_transform_output->points[0].z);
  EXPECT_FLOAT_EQ(half_sqrt2, transform_output->points[0].x);
  EXPECT_FLOAT_EQ(half_sqrt2, transform_output->points[0].y);
  EXPECT_FLOAT_EQ(transform.translation().z(), transform_output->points[0].z);
}
}  // end of namespace tmc_map_merger

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
