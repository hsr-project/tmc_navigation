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
#ifndef TMC_POSE_2D_LIB_POSE_2D_HPP_
#define TMC_POSE_2D_LIB_POSE_2D_HPP_

#include <math.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace tmc_pose_2d_lib {

double NormalizeAngle(const double angle);

/// @brief 2D xy coordinates (x[m], y[m])
class Point2d {
 private:
  double x_;
  double y_;

 public:
  explicit Point2d(const double x = 0.0, const double y = 0.0);
  double x() const {return x_;}
  double y() const {return y_;}
  void set_x(const double x) {x_ = x;}
  void set_y(const double y) {y_ = y;}
  double norm() const;
  double norm_square() const;

  Point2d operator+(const Point2d& rhs) const;
  Point2d operator-(const Point2d& rhs) const;
  Point2d operator-() const;
  Point2d operator*(const double k) const;
  Point2d& operator*=(const double k);
  bool operator==(const Point2d& other) const;
};

Point2d operator*(const double k, const Point2d& point);

// @brief For display
std::ostream &operator<<(std::ostream &out, const Point2d& p);

using Cloud2d = std::vector<Point2d>;

class Rotation2d {
 private:
  double theta_;

 public:
  explicit Rotation2d(const double theta = 0.0);
  double theta() const {return theta_;}
  void set_theta(const double theta) {theta_ = theta;}

  Rotation2d Inverse() const;

  Rotation2d operator*(const Rotation2d& rhs) const;
  Point2d operator*(const Point2d& point) const;
  bool operator==(const Rotation2d& other) const;
};

Rotation2d operator*(const double k, const Rotation2d& rot);
std::ostream &operator<<(std::ostream &out, const Rotation2d& r);

/// @brief 2D coordinate system (x[m], y[m], theta[rad])
class Pose2d {
 private:
  Rotation2d rot_;
  Point2d point_;

 public:
  explicit Pose2d(const double x = 0.0, const double y = 0.0, const double theta = 0.0);
  Pose2d(const Rotation2d r, const Point2d p);

  Point2d point() const {return point_;}
  Rotation2d rot() const {return rot_;}
  double x() const {return point_.x();}
  double y() const {return point_.y();}
  double theta() const {return rot_.theta();}

  void set_point(const Point2d p) {point_ = p;}
  void set_rot(const Rotation2d r) {rot_ = r;}
  void set_x(const double x) {point_.set_x(x);}
  void set_y(const double y) {point_.set_y(y);}
  void set_theta(const double theta) {rot_.set_theta(theta);}

  Pose2d Inverse() const;

  // Coordinate transformation of points
  Point2d operator*(const Point2d& rhs) const;
  // Composite transformation of coordinate systems
  Pose2d operator*(const Pose2d& rhs) const;
  // Coordinate transformation of point cloud
  Cloud2d operator*(const Cloud2d& points) const;
  bool operator==(const Pose2d& other) const;
};

// For display
std::ostream &operator<<(std::ostream &out, const Pose2d& pose);

}  // namespace tmc_pose_2d_lib

#endif  // TMC_POSE_2D_LIB_POSE_2D_HPP_
