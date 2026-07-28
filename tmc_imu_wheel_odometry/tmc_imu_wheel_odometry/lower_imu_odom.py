#!/usr/bin/env python3
# Copyright (c) 2026 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# -*- coding: utf-8 -*-

# IMU odometry calculation (utilizing cart information)
# Calculate the position of the lower opposing two-wheel cart of the offset cart that constitutes the omnidirectional cart of the HSR,
# integrate it with the IMU, and determine the position of the upper part (upper).
import math
from math import cos, sin

from control_msgs.msg import JointTrajectoryControllerState
import message_filters
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf_transformations

from tmc_imu_wheel_odometry import utils


##
# @brief Normalize angle (rad) to (-pi to pi)
#
# @param a Angle [rad]
#
# @return Normalized angle [rad]
def normalize_angle(a):
    b = a
    while math.fabs(b) > math.pi:
        if b > math.pi:
            b -= math.pi * 2.0
        elif b < -math.pi:
            b += math.pi * 2.0
    return b


##
# @brief Convert a 2D position and orientation list (array) to nav_msg/Odometry
#
# @param pose Position and orientation (X[m], Y[m], Yaw[rad]) (list, array)
# @param stamp Timestamp (time)
# @param frame_id frame_id (str)
# @param child_frame_id child_frame_id (str)
#
# @return Odometry msg (nav_msg/Odometry)
def pose_2_odom(pose, stamp, frame_id, child_frame_id):
    odom_msg = Odometry()

    # position
    odom_msg.pose.pose.position.x = pose[0]
    odom_msg.pose.pose.position.y = pose[1]
    odom_msg.pose.pose.position.z = 0.0

    # orientation
    q = tf_transformations.quaternion_from_euler(0, 0, pose[2])
    odom_msg.pose.pose.orientation.x = q[0]
    odom_msg.pose.pose.orientation.y = q[1]
    odom_msg.pose.pose.orientation.z = q[2]
    odom_msg.pose.pose.orientation.w = q[3]

    odom_msg.header.stamp = stamp
    odom_msg.header.frame_id = frame_id
    odom_msg.child_frame_id = child_frame_id

    return odom_msg


##
# @brief
#
# Convert a list of 2D position and orientation standard deviations to a list of 3D position and orientation variances
#
# (geometry_msgs/TwistWithCovariance/covariance)
#
# @param vel_std
#
# [Standard deviation of velocity in X direction [m/s], standard deviation of velocity in Y direction [m/s], standard deviation of velocity in Yaw direction [rad/s]]
#
#
# @return
#
# twist_covariance: geometry_msgs/TwistWithCovariance/covariance.
#
# Contents are a 36-dimensional list.
#
# Variances for Z, roll, and pitch are 0. All covariances are also 0.
def to_twist_covariance(vel_std):
    twist_covariance = np.zeros(36)
    twist_covariance[0] = vel_std[0] ** 2
    twist_covariance[7] = vel_std[1] ** 2
    twist_covariance[35] = vel_std[2] ** 2

    return twist_covariance


##
# @brief Odom Calculator that use wheel encoder and IMU
class LowerImuOdom(Node):
    ##
    # @brief Initialize paramater and ROS I/O
    #
    def __init__(self):
        u"""Initialize LowerImuOdom"""
        # Geometric configuration of the cart and upper part
        super().__init__('lower_imu_odom')
        self.wheel_radius_l = self.declare_parameter(
            'wheel_radius_l', 0.1).value
        self.wheel_radius_r = self.declare_parameter(
            'wheel_radius_r', 0.1).value
        self.caster_offset = self.declare_parameter(
            'caster_offset', 0.0).value
        self.tread = self.declare_parameter(
            'tread', 0.3).value

        # Abnormal values are around 4000, so by setting a value (100000) above that,
        # all values are adopted as default.
        self.valid_joint_vel_ths = self.declare_parameter(
            'valid_joint_vel_ths', [100000.0, 100000.0, 100000.0]).value

        # Offset angle [rad] between the cart and the wheels
        self.base_angle_offset = self.declare_parameter(
            'base_angle_offset', 0.0).value

        # Frame Name
        self.odom_link_name = self.declare_parameter(
            'odom_name', 'odom').value
        self.base_link_name = self.declare_parameter(
            'base_link_name', 'base_footprint').value

        # Standard deviation of velocity
        self.vel_std = self.declare_parameter(
            'vel_std', [0.0, 0.0, 0.0]).value

        # Position and orientation of the upper part (X[m], Y[m], Yaw[rad])
        # The position of the upper part starts from 0.
        self.current_odom_upper = np.array([0.0, 0.0, 0.0])
        # Position and orientation of the cart (X[m], Y[m], Yaw[rad])
        # The initial position of the cart is calculated from the joint axes of the turning axis.
        self.current_odom_lower = None

        self.last_integrate_time = None
        self.init_imu_yaw = None

        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry, 'imu_odom', 1)

        # Subscriber
        imu_sub = message_filters.Subscriber(self, Imu, 'imu', qos_profile=10)
        joint_state_sub = message_filters.Subscriber(
            self, JointTrajectoryControllerState, 'joint_state',
            qos_profile=10)

        # Time synchronizer
        ts = message_filters.ApproximateTimeSynchronizer(
            [imu_sub, joint_state_sub], queue_size=10, slop=0.02)
        ts.registerCallback(self.imu_joint_cb)

    ##
    # @brief Fuse wheel odometry and IMU roration factor then publish as odom
    #
    # @param imu_msg Imu
    # @param joint_msg JointTrajectoryControllerState
    #
    # @return None
    def imu_joint_cb(self, imu_msg, joint_msg):
        # It was discovered that very rarely (once every few hours), encoder values jump (around 4000 rad/sec), so those are filtered out.
        # If the joint velocity exceeds th, return.
        # TODO(nobuyuki_matsuno): 本質対策がされたら本設定を削除
        if (abs(joint_msg.feedback.velocities[0]) > self.valid_joint_vel_ths[0]) or \
           (abs(joint_msg.feedback.velocities[1]) > self.valid_joint_vel_ths[1]) or \
           (abs(joint_msg.feedback.velocities[2]) > self.valid_joint_vel_ths[2]):
            self.get_logger().warn(
                'Too big joint velocity! [%f, %f, %f]' %
                (joint_msg.feedback.velocities[0],
                 joint_msg.feedback.velocities[1],
                 joint_msg.feedback.velocities[2]))
            return

        if self.init_imu_yaw is None:
            # Initialization of IMU angle
            q = imu_msg.orientation
            (_, _, yaw) = tf_transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w])
            self.init_imu_yaw = yaw
            self.get_logger().info('Init lower imu odometry')
            self.last_integrate_time = joint_msg.header.stamp

            # Calculate the initial position of the cart so that the initial position of the upper part becomes 0
            # (Calculate the offset between the two)
            yaw_joint_angle = -joint_msg.feedback.positions[2]
            yaw_joint_angle = normalize_angle(yaw_joint_angle)
            self.current_odom_lower = np.array([0.0, 0.0, 0.0])
            self.current_odom_lower[0] = \
                self.current_odom_upper[0] \
                - self.caster_offset * cos(-yaw_joint_angle)
            self.current_odom_lower[1] = \
                self.current_odom_upper[1] \
                - self.caster_offset * sin(-yaw_joint_angle)
            self.current_odom_lower[2] = \
                self.current_odom_upper[2] \
                - self.base_angle_offset
            return

        dt = utils.time_to_sec(joint_msg.header.stamp) - utils.time_to_sec(
            self.last_integrate_time)
        if dt <= 0.0:
            self.get_logger().warn('dt is not positive!')
            return
        self.last_integrate_time = joint_msg.header.stamp

        # Yaw of the IMU
        q = imu_msg.orientation
        (_, _, yaw_imu) = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        yaw_imu -= self.init_imu_yaw

        # Rotation amount of the turning axis [rad]
        yaw_joint_angle = -joint_msg.feedback.positions[2]
        yaw_joint_angle = normalize_angle(yaw_joint_angle)

        # Yaw of the cart [rad], obtained by subtracting the turning axis amount from the IMU angle
        self.current_odom_lower[2] = \
            yaw_imu - yaw_joint_angle - self.base_angle_offset

        # Angular velocity of the wheels [rad]
        vl_omega = joint_msg.feedback.velocities[0]
        vr_omega = joint_msg.feedback.velocities[1]

        # Velocity of the wheels [m/s]
        vl = vl_omega * self.wheel_radius_l
        vr = vr_omega * self.wheel_radius_r

        # Velocity of the cart (base_footprint coordinate system)
        v = (vr + vl) * 0.5  # Translational velocity [m/s]
        # w = (vr - vl) / self.tread  # Rotational velocity [rad]

        # Velocity of the cart (odom coordinate system)
        vx = v * math.cos(self.current_odom_lower[2])
        vy = v * math.sin(self.current_odom_lower[2])

        # Calculate the position of the cart (odom coordinate system) (integration)
        self.current_odom_lower[0] += vx * dt
        self.current_odom_lower[1] += vy * dt

        # Calculate the position of the upper part (odom coordinate system) (offset calculation)
        self.current_odom_upper[0] = \
            self.current_odom_lower[0] \
            + self.caster_offset * cos(self.current_odom_lower[2])
        self.current_odom_upper[1] = \
            self.current_odom_lower[1] \
            + self.caster_offset * sin(self.current_odom_lower[2])
        self.current_odom_upper[2] = yaw_imu

        out_msg = pose_2_odom(
            self.current_odom_upper, joint_msg.header.stamp,
            self.odom_link_name, self.base_link_name)

        # Calculate the velocity of the upper part
        theta = -(yaw_joint_angle + self.base_angle_offset)
        out_msg.twist.twist.linear.x = v * math.cos(theta)
        out_msg.twist.twist.linear.y = v * math.sin(theta)
        out_msg.twist.twist.angular.z = imu_msg.angular_velocity.z

        out_msg.twist.covariance = to_twist_covariance(self.vel_std)

        # Publish the Odometry of the upper part
        self.odom_pub.publish(out_msg)
