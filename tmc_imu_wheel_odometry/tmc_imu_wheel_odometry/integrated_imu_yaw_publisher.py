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
# Calculate yaw angle by simply integrating the yaw angular velocity of the IMU

from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf_transformations

from tmc_imu_wheel_odometry import utils


##
# @brief Integration of IMU angular_velocity.z then publish
class IntegratedImuYawPublisher(Node):
    ##
    # @brief Initialize last integrated time and yaw
    #
    def __init__(self):
        u"""Initialize IntegratedImuYawPublisher"""
        super().__init__('integrated_imu_yaw_publisher')
        self.last_integrate_time = None  # [sec]
        self.yaw = 0.0  # [rad]

        # Publishers
        self.imu_pub = self.create_publisher(
            Imu, '/integrated_imu', 10)

        # Subscriber
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_cb, 10)

    ##
    # @brief Integrate Yaw element of Imu
    #
    # @param msg sensor_msgs.msg.Imu
    #
    # @return None
    def imu_cb(self, msg):
        if self.last_integrate_time is None:
            self.last_integrate_time = msg.header.stamp
            self.get_logger().info('Init imu integrator')
            return

        dt = utils.time_to_sec(msg.header.stamp) - utils.time_to_sec(
            self.last_integrate_time)
        if dt <= 0.0:
            self.get_logger().warn('dt is not positive!')
            return
        self.last_integrate_time = msg.header.stamp

        self.yaw += msg.angular_velocity.z * dt

        current_q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        out_msg = msg
        out_msg.orientation.x = current_q[0]
        out_msg.orientation.y = current_q[1]
        out_msg.orientation.z = current_q[2]
        out_msg.orientation.w = current_q[3]
        self.imu_pub.publish(out_msg)
