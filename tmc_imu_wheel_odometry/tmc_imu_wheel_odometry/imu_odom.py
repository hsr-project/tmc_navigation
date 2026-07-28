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
# IMU odometry calculation
# Use wheel odometry for translation and IMU yaw angle for rotation, with simple integration
import math

import message_filters
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf_transformations

from tmc_imu_wheel_odometry import utils


##
# @brief Publish odometry that integrated wheel encoder and IMU yaw element
class ImuOdom(Node):
    ##
    # @brief Initialize ImuOdom
    #
    def __init__(self):
        super().__init__('imu_odom')
        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry, 'imu_odom', 10)

        # Subscriber
        imu_sub = message_filters.Subscriber(
            self, Imu, 'imu', qos_profile=10)
        odom_sub = message_filters.Subscriber(
            self, Odometry, 'wheel_odom', qos_profile=10)

        # sync topics
        ts = message_filters.ApproximateTimeSynchronizer(
            [imu_sub, odom_sub], queue_size=10, slop=0.02)
        ts.registerCallback(self.imu_odom_cb)

        self.integrated_odom = Odometry()
        self.last_integrate_time = None  # [sec]
        self.init_yaw = None  # [rad]

    ##
    # @brief Publish integrated odometry
    #
    # @param imu_msg Imu
    # @param odom_msg Odometry
    #
    # @return None
    def imu_odom_cb(self, imu_msg, odom_msg):
        if self.init_yaw is None:
            q = imu_msg.orientation
            (_, _, yaw) = tf_transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w])
            self.init_yaw = yaw
            self.last_integrate_time = odom_msg.header.stamp
            self.get_logger().info('Init imu odometry')
            return

        dt = utils.time_to_sec(odom_msg.header.stamp) - utils.time_to_sec(
            self.last_integrate_time)
        if dt <= 0.0:
            self.get_logger().warn('dt is not positive!')
            return
        self.last_integrate_time = odom_msg.header.stamp

        # Get yaw angle of IMU
        q = imu_msg.orientation
        (_, _, yaw) = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        yaw -= self.init_yaw

        # calc global velocity
        twist = odom_msg.twist.twist
        vx = twist.linear.x * math.cos(yaw) - twist.linear.y * math.sin(yaw)
        vy = twist.linear.y * math.cos(yaw) + twist.linear.x * math.sin(yaw)

        # Integration of X, Y
        self.integrated_odom.pose.pose.position.x += vx * dt
        self.integrated_odom.pose.pose.position.y += vy * dt

        # Use IMU orientation
        current_q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        self.integrated_odom.pose.pose.orientation.x = current_q[0]
        self.integrated_odom.pose.pose.orientation.y = current_q[1]
        self.integrated_odom.pose.pose.orientation.z = current_q[2]
        self.integrated_odom.pose.pose.orientation.w = current_q[3]

        # Twist
        # For vx, vy, use wheel odometry values; for v_theta, use IMU angular velocity
        self.integrated_odom.twist = odom_msg.twist
        self.integrated_odom.twist.twist.angular.z = imu_msg.angular_velocity.z

        self.integrated_odom.header = odom_msg.header

        self.odom_pub.publish(self.integrated_odom)
