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
/// @file     laser_2d_mcl.hpp
/// @brief    Self-localization using 2D point cloud
/// @version  0.x.x
/// @author   Yoshiaki Asahara
/// @date     2011.xx.xx
/// @since    2011.xx.xx
/// @note     Applied for Partner-Robot Coding Rule(Ver:x.xx)
#ifndef TMC_LASER_2D_LOCALIZER_LASER_2D_MCL_HPP_
#define TMC_LASER_2D_LOCALIZER_LASER_2D_MCL_HPP_

#include <memory>
#include <string>
#include <vector>
#include <tmc_pose_2d_lib/distance_map.hpp>
#include <tmc_pose_2d_lib/pose_2d.hpp>

extern "C" {
#include "laser_2d_mcl_lib.h"
}

namespace tmc_laser_2d_localizer {
using tmc_pose_2d_lib::DistanceMap;
using tmc_pose_2d_lib::Point2d;
using tmc_pose_2d_lib::Pose2d;
using tmc_pose_2d_lib::Rotation2d;
/// Parameters
struct Laser2dMclParams {
  /// Number of particles for tracking
  double number_of_particles;
  /// Threshold for the number of effective particles
  double effective_particle_ratio;
  /// Standard deviation in the y(x) axis direction for the x(y) coordinate of movement
  double standard_deviation_xy_to_yx;
  /// Standard deviation in the x(y) axis direction for the x(y) coordinate of movement
  double standard_deviation_xy_to_xy;
  /// Standard deviation in the θ coordinate for the translational distance of movement
  double standard_deviation_xy_to_theta;
  /// Standard deviation in the x(y) axis direction for the θ coordinate of movement
  double standard_deviation_theta_to_xy;
  /// Standard deviation in the θ axis direction for the θ coordinate of movement
  double standard_deviation_theta_to_theta;
  /// Standard deviation for the x(y) coordinate of the robot's initial position
  double standard_deviation_init_xy;
  /// Standard deviation for the θ coordinate of the robot's initial position
  double standard_deviation_init_theta;
  /// Initial position and orientation
  Pose2d init_pose;
  /// Distance threshold for executing the particle filter
  double distance_triggering_filter;
  /// Angle threshold for executing the particle filter
  double angle_triggering_filter;
  /// width of the potential field representing distance.
  double potential_width;
  /// At what distance from the wall is an obstacle considered not on the map?
  double filtering_thresh;
};


/**
  @brief  Laser2dMcl class
  Create self-localization data using 2D LRF data
  @par  Input
  odometry_    Odometry information
  point_cloud2_  2D LRF data
  @par  Output
  laser_2d_pose_    2D LRF self-localization data
  @attention    If the initial position is not specified, the coordinate reference point will be unknown.
*/
class Laser2dMcl {
 public:
  Laser2dMcl();
  ~Laser2dMcl();
  void set_params(const Laser2dMclParams& value);
  Laser2dMclParams params() const { return params_; }
  Laser2dMclParams default_params() const { return default_params_; }
  void set_init_pose_param(const Pose2d& init_pose);
  Pose2d init_pose_param() const;
  void set_odometry(const Pose2d& value);
  void set_distance_map(const std::shared_ptr<DistanceMap>& value);
  void set_point_cloud_2d(const icSlam_tagRangeXY& value);
  void set_initial_pose(const Pose2d& value);
  void set_localization_score_limit(const double& value) { localization_score_limit_ = value; }
  void set_is_manual_reset(const bool value) { is_manual_reset_ = value; }
  double localization_score_limit(void) { return localization_score_limit_; }
  void set_no_noise_model(void) { is_no_noise_ = true; }
  Pose2d CorrectPosition();
  std::vector<Point2d> GetParticlePositions();
  icSlam_tagRangeXY FilterPointCloud(const icSlam_tagRangeXY& point_cloud, double interval);
  double GetMapResolution() const;
  bool IsMapInitialized() const;
  double GetScore() { return best_matching_score_; }

 private:
  /// Latest odometry
  Pose2d odometry_;
  /// First odometry received immediately after startup
  Pose2d init_odometry_;
  /// Parameters set by the client
  Laser2dMclParams params_;
  /// Default parameters
  Laser2dMclParams default_params_;
  /// Point cloud
  const icSlam_tagRangeXY* p_point_cloud_2d_;
  /// Map data
  std::shared_ptr<DistanceMap> p_distance_map_;
  /// Odometry initialized flag
  bool is_odometry_initialized_;
  /// Point cloud initialized flag
  bool is_range_data_initialized_;
  /// Self-localization reset flag
  /// @par Becomes true when self-localization is reset externally
  /// Unlike other flags, it may toggle between true and false during program execution
  bool is_pose_initialized_;
  /// Self-localization result
  Pose2d laser_2d_pose_;
  /// Previous self-localization result
  Pose2d previous_laser_2d_pose_;
  /// Best score of map matching
  double best_matching_score_;
  /// Score threshold for map matching
  double localization_score_limit_;
  /// Whether it is a correction by laser_2d_correct_pose
  bool is_manual_reset_;
  bool is_no_noise_;
};
}  // namespace tmc_laser_2d_localizer
#endif  // TMC_LASER_2D_LOCALIZER_LASER_2D_MCL_HPP_
