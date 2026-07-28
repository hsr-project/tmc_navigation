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
/// @file     pose_integrator.hpp
/// @brief    Integrates multiple self-localization results (library header)
/// @version  0.2.0
/// @author   Takao Yasuda
/// @author   Applied for Partner-Robot Coding Rule(Ver:x.xx)
/// @date     2012.05.08

#ifndef TMC_POSE_INTEGRATOR_POSE_INTEGRATOR_HPP_
#define TMC_POSE_INTEGRATOR_POSE_INTEGRATOR_HPP_

#include <stdint.h>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>

/// Namespace (tmc_pose_integrator)
namespace tmc_pose_integrator {

/// Definition of immutable parameters
uint32_t const kCovarianceMatrixSize36 = 36;  // Number of elements in the covariance matrix

/// Structure for robot self-localization
struct Pose2d {
  /// Constructor
  Pose2d() {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    time = 0.0;
  }
  /// Zero clear
  void ZeroClear() {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    time = 0.0;
  }
  /// Coordinate X (m)
  double x;
  /// Coordinate Y (m)
  double y;
  /// Orientation angle (rad)
  double theta;
  /// Timestamp (sec)
  double time;
};

/// Structure for self-localization estimation information using 2D laser data
struct Pose2dWithCovariance {
  /// Constructor
  Pose2dWithCovariance() {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    time = 0.0;
    for (uint32_t i = 0; i < kCovarianceMatrixSize36; i++) {
      covariance[i] = 0.0;
    }
  }
  /// Zero clear
  void ZeroClear() {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    time = 0.0;
    for (uint32_t i = 0; i < kCovarianceMatrixSize36; i++) {
      covariance[i] = 0.0;
    }
  }
  /// Coordinate X (m)
  double x;
  /// Coordinate Y (m)
  double y;
  /// Orientation angle (rad)
  double theta;
  /// Timestamp (sec)
  double time;
  /// Covariance
  double covariance[kCovarianceMatrixSize36];
};

/// Self-localization integration class
/// @todo Add methods using 3D data obtained from lasers and cameras in the future
class PoseIntegrator {
 public:
  typedef boost::shared_ptr<PoseIntegrator> Ptr;
  /// Constructor
  PoseIntegrator();
  /// Odometry set. Not inline due to the large number of variables to initialize.
  void set_odometry(const Pose2d& value);
  /// Set odometry synchronized with self-localization
  void set_synchronized_odometry(const Pose2d& value);
  /// Set 2D LRF self-localization estimation.
  /// Not inline because it also manipulates update flags.
  void set_localized_2d_pose(const Pose2dWithCovariance& value);
  /// Set convergence time for self-localization calculation using linear convergence
  void set_convergence_time(double value);
  /// Set client operation cycle used for convergence calculation
  void set_cycle_time(double value);
  void set_stop_translational_vel(double value) { stop_translational_vel_ = value; }
  void set_stop_rotational_vel(double value) { stop_rotational_vel_ = value; }
  /// Indicates whether odometry has been acquired after startup
  /// @return true: acquired, false: not acquired
  bool is_first_odometry_received() { return is_first_odometry_received_; }
  /// Integration of self-localization estimation using linear convergence
  Pose2d CorrectOdometryWithConvergence();
  /// Self-localization estimation using time-synchronized odometry
  Pose2d CorrectOdometryWithSynchronizedOdometry();
  Pose2d CorrectOdometryWithConvergenceAndSynchronization();
  /// Determine whether the vehicle is moving based on odometry
  bool IsBaseMoving();

 private:
  /// Odometry variable initialization completion flag.
  /// @par Used to check whether initialization of odometry-related members is complete.
  bool is_first_odometry_received_;
  /// Update confirmation flag for self-localization estimation
  bool is_localization_updated_;
  /// Unit: s. Convergence time parameter used during self-localization convergence
  double convergence_time_;
  /// Unit: s. Client operation cycle.
  /// @todo Used for convergence calculation of self-localization. Assumes periodic operation,
  /// @par Measuring time internally within the object would provide greater flexibility. Issue.
  double cycle_time_;
  /// Time (s) elapsed since receiving external self-localization estimation results
  double time_from_pose_reset_;
  /// Input: Latest odometry data
  Pose2d odometry_;
  /// Reference odometry at the moment self-localization correction was subscribed
  Pose2d odometry_at_localization_update_;
  /// Current corrected odometry
  Pose2d corrected_odometry_;
  /// Self-localization at the time laser self-localization was received
  Pose2d corrected_odometry_at_localization_;
  /// Odometry movement amount since the time laser self-localization was received
  Pose2d diff_odometry_;
  /// Previously acquired odometry
  Pose2d old_odometry_;
  /// Time-synchronized odometry
  Pose2d synchronized_odometry_;
  /// Current correction amount
  Pose2d current_adjusted_pose_;
  /// Correction amount when laser_2d_pose was received
  Pose2d adjusted_pose_at_localization_;
  /// Target correction amount
  Pose2d target_adjusted_pose_;
  /// Input: Self-localization estimation data. Odometry is corrected towards this value.
  Pose2dWithCovariance localized_2d_pose_;
  /// Self-localization estimation value moved by the time delay
  Pose2dWithCovariance localized_2d_pose_at_localization_update_;
  /// Previous odometry when laser_2d_pose was received
  Pose2d previous_odometry_;
  /// Previous time when laser_2d_pose was received
  double previous_time_;
  /// Translational velocity considered as the vehicle being stationary
  double stop_translational_vel_;
  /// Rotational velocity considered as the vehicle being stationary
  double stop_rotational_vel_;
  /// Flag indicating whether a correction has been made even once
  bool is_first_localization_;
};
}  // namespace tmc_pose_integrator

#endif  // TMC_POSE_INTEGRATOR_POSE_INTEGRATOR_HPP_
