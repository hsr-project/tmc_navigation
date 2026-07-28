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
import test.ros_test_utils as ros_test_utils
from threading import Lock, Thread
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import Imu
import tf_transformations


@pytest.mark.launch_test
def generate_test_description():
    integrated_imu_yaw_publisher_node = launch_ros.actions.Node(
        package='tmc_imu_wheel_odometry',
        executable='integrated_imu_yaw_publisher_node',
        output='screen'
    )
    return launch.LaunchDescription(
        [integrated_imu_yaw_publisher_node, launch_testing.actions.ReadyToTest()])


class IntegratedImuYawPublisherTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._node = rclpy.create_node("test_integrated_imu_yaw_publisher")
        self.lock = Lock()
        self.last_imu_msg = None
        self.publish_rate = 10  # hz

        # publisher
        self.pub_imu_ = ros_test_utils.VariableImu(
            self._node, topic='imu', rate_hz=self.publish_rate)

        # Subscriber
        self.sub_imu_ = self._node.create_subscription(
            Imu, 'integrated_imu', self.imu_cb, 10)
        self._executor = SingleThreadedExecutor()
        self._thread = Thread(target=rclpy.spin, args=(self._node, self._executor), daemon=True)
        self._thread.start()

    def imu_cb(self, imu_msg):
        with self.lock:
            self.last_imu_msg = imu_msg

    def connection_check(self):
        # connection test
        self.assertTrue(self.pub_imu_.check_connection())
        self.assertTrue(ros_test_utils.check_subscription_connection(self._node, self.sub_imu_))

    def test_success_case(self):
        self.connection_check()

        yaw_rate = 0.1  # rad/sec
        self.pub_imu_.set_angular_velocities(0.0, 0.0, yaw_rate)

        duration = 3.0  # sec
        self.pub_imu_.start()
        time.sleep(duration)
        self.pub_imu_.stop()

        # If you evaluate immediately after stopping publication, you may evaluate the value before the intended calculation result, so wait.
        time.sleep(1.0)
        with self.lock:
            _, _, yaw = tf_transformations.euler_from_quaternion(
                [self.last_imu_msg.orientation.x,
                 self.last_imu_msg.orientation.y,
                 self.last_imu_msg.orientation.z,
                 self.last_imu_msg.orientation.w])

        expected_yaw = yaw_rate * duration  # rad

        self._node.get_logger().warn(
            'yaw:%f, expected_yaw:%f, delta:%f, rate[percent]:%f ' %
            (yaw, expected_yaw, (yaw - expected_yaw),
             (yaw - expected_yaw) / expected_yaw * 100.0))

        self.assertAlmostEqual(yaw, expected_yaw, delta=yaw_rate / 10.0)
