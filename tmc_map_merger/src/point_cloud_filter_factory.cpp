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

#include <string>
#include <vector>

#include "param.hpp"
#include "point_cloud_filter.hpp"

namespace tmc_map_merger {
PointCloudFilter::Ptr PointCloudFilterFactory::Create(const std::map<std::string, rclcpp::Parameter>& param) {
  PointCloudFilter::Ptr ptr;
  std::string type;
  GetRequiredParam(param, "type", type);
  if (type == "voxel_grid") {
    double leaf_size;
    GetOptionalParam(param, "leaf_size", leaf_size, 0.05, NotLess<double>(0.0));
    Eigen::Vector3d leaf_size_eigen;
    leaf_size_eigen << leaf_size, leaf_size, leaf_size;

    ptr.reset(new PointCloudVoxelGridFilter(leaf_size_eigen));
  } else if (type == "trimming") {
    // Do not accept filtering of fields other than x, y, z
    std::vector<std::string> field_names;
    field_names.push_back("x");
    field_names.push_back("y");
    field_names.push_back("z");
    std::string field_name;
    GetOptionalParam(param, "field_name", field_name, std::string("z"), OneOf<std::string>(field_names));
    double min;
    GetOptionalParam(param, "min", min, 0.15, NotLess<double>(0.0));
    // The upper limit of the trimming range must be greater than the lower limit
    double max;
    GetOptionalParam(param, "max", max, min * 2, NotLess<double>(min));

    ptr.reset(new PointCloudTrimmingFilter(field_name, min, max));
  } else if (type == "noise") {
    double radius;
    GetOptionalParam(param, "radius", radius, 0.1, NotLess<double>(0.0));
    int min_neighbors;
    GetOptionalParam(param, "min_neighbors", min_neighbors, 1, NotLess<int>(0));

    ptr.reset(new PointCloudNoiseFilter(radius, min_neighbors));
  } else if (type == "transform") {
    ptr.reset(new PointCloudTransformFilter());
  } else {
    const std::string message = std::string("Unknown filter type: ") + type;
    throw std::logic_error(message);
  }
  return ptr;
}
}  // end of namespace tmc_map_merger
