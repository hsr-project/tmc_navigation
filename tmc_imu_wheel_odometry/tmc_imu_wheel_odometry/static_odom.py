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
# Publish odometry for complete standstill

from nav_msgs.msg import Odometry
from rclpy.node import Node

DefaultRate = 30.0  # [Hz]
DefaultFrameId = 'odom'
DefaultChildFrameId = 'base_footprint'


##
# @brief Generate Zero odometry msg
#
# @param frame_id frame_id(str)
# @param child_frame_id child_frame_id(str)
#
# @return Odometry
def create_static_odom(frame_id, child_frame_id):
    odom = Odometry()
    odom.header.frame_id = frame_id
    odom.child_frame_id = child_frame_id
    odom.pose.pose.position.x = 0.0
    odom.pose.pose.position.y = 0.0
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation.x = 0.0
    odom.pose.pose.orientation.y = 0.0
    odom.pose.pose.orientation.z = 0.0
    odom.pose.pose.orientation.w = 1.0
    return odom


##
# @brief Const zero odom publisher
class StaticOdom(Node):
    ##
    # @brief InitializeROS I/O
    #
    def __init__(self):
        super().__init__('static_odom')
        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry, 'static_odom', 1)

        # publish rate
        rate = self.declare_parameter('rate', DefaultRate).value

        # frame_id and child_frame id of message
        frame_id = self.declare_parameter('frame_id', DefaultFrameId).value
        child_frame_id = self.declare_parameter(
            'child_frame_id', DefaultChildFrameId).value

        self.odom = create_static_odom(frame_id, child_frame_id)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    ##
    # @brief Cyclic publishing
    #
    # @return None
    def timer_callback(self):
        self.publish_static_odom()

    ##
    # @brief Once publish message
    #
    # @return None
    def publish_static_odom(self):
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(self.odom)
