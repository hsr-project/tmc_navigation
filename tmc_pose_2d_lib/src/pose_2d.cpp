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
#include "tmc_pose_2d_lib/pose_2d.hpp"
#include <climits>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

namespace tmc_pose_2d_lib {

double NormalizeAngle(const double angle) {
  double angle_out = angle;
  while (fabs(angle_out) > M_PI) {
    if (angle_out >= M_PI) {
        angle_out -= 2.0 * M_PI;
    } else if (angle_out <= -M_PI) {
        angle_out += 2.0 * M_PI;
    }
  }
  return angle_out;
}


Point2d::Point2d(const double x, const double y): x_(x), y_(y) {}

double Point2d::norm() const {
  return std::sqrt(norm_square());
}

double Point2d::norm_square() const {
  return x_*x_ + y_*y_;
}

Point2d Point2d::operator+(const Point2d& rhs) const {
  return Point2d(x_ + rhs.x(), y_ + rhs.y());
}

Point2d Point2d::operator*(const double k) const {
  return std::move(Point2d(k * x_, k * y_));
}
Point2d& Point2d::operator*=(const double k) {
  *this = Point2d(k * x_, k * y_);
  return *this;
}

Point2d Point2d::operator-(const Point2d& rhs) const {
  return std::move(Point2d(x_ - rhs.x(), y_ - rhs.y()));
}

Point2d Point2d::operator-() const {
  return std::move(Point2d(-x_, -y_));
}

bool Point2d::operator==(const Point2d& other) const {
  return (fabs(x_ - other.x()) < std::numeric_limits<double>::epsilon() &&
          fabs(y_ - other.y()) < std::numeric_limits<double>::epsilon());
}

Point2d operator*(const double k, const Point2d& point) {
  return std::move(Point2d(k * point.x(), k * point.y()));
}

std::ostream &operator<<(std::ostream &out, const Point2d& p) {
  out << "[" << p.x() << ", " << p.y() << "]" << std::endl;
  return out;
}

Rotation2d::Rotation2d(const double theta): theta_(NormalizeAngle(theta)) {
}

Rotation2d Rotation2d::Inverse() const {
  return std::move(Rotation2d(-theta_));
}

Rotation2d operator*(const double k, const Rotation2d& rot) {
  return std::move(Rotation2d(k * rot.theta()));
}

Rotation2d Rotation2d::operator*(const Rotation2d& rhs) const {
  return std::move(Rotation2d(theta_ + rhs.theta()));
}

Point2d Rotation2d::operator*(const Point2d& point) const {
  double sin_theta, cos_theta;
  sincos(theta_, &sin_theta, &cos_theta);
  return std::move(Point2d(cos_theta * point.x() - sin_theta * point.y(),
                           sin_theta * point.x() + cos_theta * point.y()));
}
bool Rotation2d::operator==(const Rotation2d& other) const {
  return fabs(theta_ - other.theta()) < std::numeric_limits<double>::epsilon();
}

std::ostream &operator<<(std::ostream &out, const Rotation2d& r) {
  out << "[ " << r.theta() << "[rad], " << r.theta()/M_PI * 180.0 << "[deg] ]" << std::endl;
  return out;
}

Pose2d::Pose2d(const double x, const double y, const double theta):
  rot_(Rotation2d(theta)), point_(Point2d(x, y)) {
}

Pose2d::Pose2d(const Rotation2d r, const Point2d p): rot_(r), point_(p) {
}

/// @brief Inverse transformation
Pose2d Pose2d::Inverse() const {
  return std::move(Pose2d(rot_.Inverse(), rot_.Inverse() * (-point_)));
}

// Coordinate transformation of a point
Point2d Pose2d::operator*(const Point2d& point) const {
  return rot_ * point + point_;
}

// Composite coordinate transformation
Pose2d Pose2d::operator*(const Pose2d& rhs) const {
  const Pose2d pose(*this);
  return std::move(Pose2d(rot_ * rhs.rot(), pose * rhs.point()));
}

// Coordinate transformation of a point cloud
Cloud2d Pose2d::operator*(const Cloud2d& points) const {
  Cloud2d moved_points(points.size());
  for (size_t i=0; i < points.size(); i++) {
    moved_points[i] = (*this) * points[i];
  }
  return moved_points;
}

bool Pose2d::operator==(const Pose2d& other) const {
  return (point_ == other.point() && rot_ == other.rot());
}

std::ostream &operator<<(std::ostream &out, const Pose2d& pose) {
  out << "[" << pose.x() << ", " << pose.y() << ", " << pose.theta() << "[rad] ]" << std::endl;
  return out;
}

}  // namespace tmc_pose_2d_lib
