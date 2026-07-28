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

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


class PointCloudMergerTestNode(Node):
    def __init__(self):
        super().__init__('point_cloud_merger_test')
        # Test point cloud
        self._point1 = [[0.5, 0.5, 0.0]]
        self._point2 = [[1.0, 1.0, 0.0], [1.5, 1.5, 0.0]]
        # Whether the merged point cloud is received
        self._is_merged_cloud_subscribed = False
        # Execution cycle (Hz)
        test_rate = 10.0
        self._rate = self.create_rate(test_rate)

        # Publishers
        self._cloud1_publisher = self.create_publisher(PointCloud2, 'input_1', 1)
        self._cloud2_publisher = self.create_publisher(PointCloud2, 'input_2', 1)

        # Subscriber
        self._merged_cloud_subscriber = self.create_subscription(PointCloud2, '/merged_point_cloud',
                                                                 self._merged_cloud_callback, 1)

        # timer
        self.timer = self.create_timer(1.0 / test_rate, self._timer_callback)

    def _create_cloud(self, timestamp, frame_id, point):
        header = Header()
        header.frame_id = frame_id
        header.stamp = timestamp
        return pc2.create_cloud_xyz32(header, point)

    def _publish_cloud(self, frame1='frame1', frame2='frame2'):
        timestamp = self.get_clock().now().to_msg()
        self._cloud1_publisher.publish(self._create_cloud(timestamp, frame1, self._point1))
        self._cloud2_publisher.publish(self._create_cloud(timestamp, frame2, self._point2))

    def _merged_cloud_callback(self, msg):
        self._merged_cloud = msg
        self._is_merged_cloud_subscribed = True

    def _timer_callback(self):
        self._publish_cloud()

    # Wait until the point cloud can be obtained
    def wait_for_merged_cloud(self, timeout):
        wait_start_time = self.get_clock().now().nanoseconds
        while (not self._is_merged_cloud_subscribed
               and (self.get_clock().now().nanoseconds - wait_start_time) < timeout * 1e9):
            self._rate.sleep()
        if not self._is_merged_cloud_subscribed:
            return False
        self._is_merged_cloud_subscribed = False
        return True

    # Return the number of points in the point cloud
    def count_cloud_points(self, cloud):
        cloud_points = list(pc2.read_points(cloud,
                                            skip_nans=True,
                                            field_names=('x', 'y', 'z')))
        return len(cloud_points)

    # Return the merged point cloud
    def merged_cloud(self):
        return self._merged_cloud
