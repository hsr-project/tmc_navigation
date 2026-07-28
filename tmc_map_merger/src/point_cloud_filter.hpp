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
#ifndef TMC_MAP_MERGER_POINT_CLOUD_FILTER_HPP_
#define TMC_MAP_MERGER_POINT_CLOUD_FILTER_HPP_
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.hpp>

typedef pcl::PointXYZ Point3f;
typedef pcl::PointCloud<Point3f> PointCloud;

namespace tmc_map_merger {

class PointCloudFilter {
 public:
  using Ptr = std::shared_ptr<PointCloudFilter>;
  virtual void Filter(const PointCloud::ConstPtr& in, const Eigen::Affine3d& transform, const PointCloud::Ptr& out) = 0;
};

class PointCloudVoxelGridFilter : public PointCloudFilter {
 public:
  PointCloudVoxelGridFilter() {
    // Set the default Voxel size [m]
    filter_.setLeafSize(0.05, 0.05, 0.05);
  }
  explicit PointCloudVoxelGridFilter(const Eigen::Vector3d& leaf_size) {
    filter_.setLeafSize(leaf_size(0, 0), leaf_size(1, 0), leaf_size(2, 0));
  }

  virtual void Filter(const PointCloud::ConstPtr& in, const Eigen::Affine3d& transform, const PointCloud::Ptr& out) {
    if (in->points.empty()) {
      out->points.clear();
      return;
    }
    filter_.setInputCloud(in);
    filter_.filter(*out);
  }

 private:
  pcl::VoxelGrid<Point3f> filter_;
};

class PointCloudTrimmingFilter : public PointCloudFilter {
 public:
  typedef pcl::PassThrough<Point3f> TrimmingFilter;
  PointCloudTrimmingFilter() {
    filter_.setFilterFieldName("z");
    filter_.setFilterLimits(0.15, 1.2);
  }
  PointCloudTrimmingFilter(const std::string& field_name, const double min, const double max) {
    filter_.setFilterFieldName(field_name);
    filter_.setFilterLimits(min, max);
  }

  virtual void Filter(const PointCloud::ConstPtr& in, const Eigen::Affine3d& transform, const PointCloud::Ptr& out) {
    if (in->points.empty()) {
      out->points.clear();
      return;
    }
    filter_.setInputCloud(in);
    filter_.filter(*out);
  }

 private:
  TrimmingFilter filter_;
};

class PointCloudNoiseFilter : public PointCloudFilter {
 public:
  typedef pcl::RadiusOutlierRemoval<Point3f> NoiseFilter;
  PointCloudNoiseFilter() {
    filter_.setRadiusSearch(0.1);
    filter_.setMinNeighborsInRadius(1);
  }
  PointCloudNoiseFilter(const double radius, const int min_neighbors) {
    filter_.setRadiusSearch(radius);
    filter_.setMinNeighborsInRadius(min_neighbors);
  }

  virtual void Filter(const PointCloud::ConstPtr& in, const Eigen::Affine3d& transform, const PointCloud::Ptr& out) {
    // This filter cannot be applied to data containing invalid values
    if (in->points.empty() || !in->is_dense) {
      out->points.clear();
      return;
    }
    filter_.setInputCloud(in);
    filter_.filter(*out);
  }

 private:
  NoiseFilter filter_;
};

class PointCloudTransformFilter : public PointCloudFilter {
 public:
  virtual void Filter(const PointCloud::ConstPtr& in, const Eigen::Affine3d& transform, const PointCloud::Ptr& out) {
    if (in->points.empty()) {
      out->points.clear();
      return;
    }
    pcl::transformPointCloud(*in, *out, transform);
  }
};

class PointCloudFilterFactory {
 public:
  static PointCloudFilter::Ptr Create(const std::map<std::string, rclcpp::Parameter>& parameters);
};

}  // end of namespace tmc_map_merger
#endif
