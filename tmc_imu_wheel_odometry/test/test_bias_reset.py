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
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor


@pytest.mark.launch_test
def generate_test_description():
    imu_reset_bias_node = launch_ros.actions.Node(
        package='tmc_imu_wheel_odometry',
        executable='imu_reset_bias_node',
        parameters=[{'reset_data_num': 250},
                    {'reset_srv_timeout': 5},
                    {'imu_angular_threshold': [0.02]}],
        remappings=[('data_raw_br', 'data_br'),
                    ('data_raw', 'data_raw'),
                    ('cmd_vel', 'command_velocity')],
        output='screen'
    )
    return launch.LaunchDescription(
        [imu_reset_bias_node, launch_testing.actions.ReadyToTest()])


class BiasResetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._node = rclpy.create_node("test_bias_reset")
        self._test_io = ros_test_utils.TestIO(self._node)
        self._executor = SingleThreadedExecutor()
        self._thread = threading.Thread(target=rclpy.spin, args=(self._node, self._executor), daemon=True)
        self._thread.start()

    def test_success_case(self):
        u"""Success bias reset case"""
        # setup
        self._test_io.connection_check()
        input_angular_vel = [0.0007, 0.0008, 0.0009]
        self._test_io.set_test_data(
            imu_angular=input_angular_vel,
            command_vel_angular=[0.0009, 0.0009, 0.0009],
            command_vel_linear=[0.0009, 0.0009, 0.0009])

        # exercise
        self._test_io.start_publish()
        time.sleep(3)
        self._test_io.stop_publish()

        # verify
        bias = self._test_io.bias()
        imu = self._test_io.imu().angular_velocity
        self.assertAlmostEqual(input_angular_vel[0], bias.x)
        self.assertAlmostEqual(input_angular_vel[1], bias.y)
        self.assertAlmostEqual(input_angular_vel[2], bias.z)
        self.assertAlmostEqual(0., imu.x)
        self.assertAlmostEqual(0., imu.y)
        self.assertAlmostEqual(0., imu.z)
