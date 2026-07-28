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
# Test to compare IMU odometry output with ground truth
import test.ros_test_utils as ros_test_utils
from threading import Lock, Thread
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_testing
from nav_msgs.msg import Odometry
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.wait_for_message import wait_for_message
from rosgraph_msgs.msg import Clock

SHOW_INFO = False


@pytest.mark.launch_test
def generate_test_description():
    # Since only type=lower supports ROS2, the test also supports only lower
    # TODO(kazuki_shibamiya) : type=lower以外に対応した場合、テストも対応する
    share_dir = get_package_share_directory('tmc_imu_wheel_odometry')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    imu_odom_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([share_dir + '/launch/imu_odom_system.launch.py']),
        launch_arguments={
            'imu_raw_topic': '/hsrb/base_accurate_imu/data_raw',
            'br_imu_topic': 'base_accurate_imu/data_br',
            'imu_topic': 'base_accurate_imu/data',
            'cmd_vel_topic': '/hsrb/command_velocity',
            'imu_bias_topic': 'base_accurate_imu/bias',
            'joint_state_topic': '/hsrb/omni_base_controller/internal_state',
            'imu_odom_topic': '/hsrb/imu_odom',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # To test without depending on the load conditions of the test environment, playback speed is reduced to 0.5x
    # To prevent the test from being unable to finish when rosbag playback ends during the test, rosbag is set to loop playback
    rosbag_play_node = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'play', './test/rosbag/extracted_20190626_1147_d_loc_naka_gt', '-r 0.5', '--clock', '-l'],
        output='screen'
    )
    return launch.LaunchDescription(
        [use_sim_time, imu_odom_system, rosbag_play_node, launch_testing.actions.ReadyToTest()])


class ImuOdomSystemTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._node = rclpy.create_node('imu_odom_system_test')
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )
        update_parameters = [use_sim_time_param]
        self._node.set_parameters(update_parameters)
        self._lock = Lock()
        self.first_odom = None
        self.last_odom = None

        self.first_gt = None
        self.last_gt = None

        self.show_info = SHOW_INFO

        # Wait until receiving the clock from rosbag,
        # Ensure the test can be conducted with the clock from rosbag
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        result, _ = wait_for_message(Clock, self._node, '/clock', qos_profile=qos_policy, time_to_wait=10)
        self.assertTrue(result)

        self._sub_odom = self._node.create_subscription(
            Odometry, '/hsrb/imu_odom', self._odom_cb, 10)

        self._sub_gt = self._node.create_subscription(
            PoseStamped, '/global_pose', self._gt_cb, 10)

        self._executor = SingleThreadedExecutor()
        self._thread = Thread(target=rclpy.spin, args=(self._node, self._executor), daemon=True)
        self._thread.start()

    def _odom_cb(self, msg):
        if self.first_odom is None:
            self.first_odom = msg
        self.last_odom = msg

    def _gt_cb(self, msg):
        if self.first_gt is None:
            self.first_gt = msg
        self.last_gt = msg

    def test_gt(self):
        u"""Compare output of IMU odometry and GT"""
        # Sometimes topics published immediately after creating a publisher/subscriber are not received
        # Add a wait at the start of the test to stabilize it
        time.sleep(1.0)
        test_duration = 20  # sec
        test_rate = 10  # hz
        rate = self._node.create_rate(test_rate)

        if self.show_info:
            max_errs = [0.0, 0.0, 0.0]  # x, y, theta

        # Wait for one cycle because the time from rosbag cannot be obtained immediately after the test starts
        rate.sleep()
        start_time = self._node.get_clock().now().nanoseconds
        last_time = 0
        while (rclpy.ok() and self._node.get_clock().now().nanoseconds - start_time < test_duration * 1e9):
            rate.sleep()
            # Confirm that rosbag has not finished playback (looped)
            now_time = self._node.get_clock().now().nanoseconds
            self.assertLessEqual(last_time, now_time)
            last_time = now_time

            if (self.first_gt is None) or (self.first_odom is None):
                continue

            # Convert to a position relative to the initial position to align coordinate systems
            d_trans, d_euler = ros_test_utils.get_mat_pose_diff(
                self.first_odom, self.last_odom
            )
            gt_d_trans, gt_d_euler = ros_test_utils.get_mat_pose_diff(
                self.first_gt, self.last_gt
            )

            if self.show_info:
                self._node.get_logger().info('------------')
                self._node.get_logger().info('x    : %6.2f, %6.2f, diff: %6.2f' % (
                    d_trans[0], gt_d_trans[0], d_trans[0] - gt_d_trans[0]))
                self._node.get_logger().info('y    : %6.2f, %6.2f, diff: %6.2f' % (
                    d_trans[1], gt_d_trans[1], d_trans[1] - gt_d_trans[1]))
                self._node.get_logger().info('theta: %6.2f, %6.2f, diff: %6.2f' % (
                    d_euler[2], gt_d_euler[2], d_euler[2] - gt_d_euler[2]))

            # Temporarily relaxed the threshold due to test instability
            # TODO(kazuhito_tanaka): テストが安定したことを確認したら閾値見直し
            delta_xy = 0.3  # m
            delta_yaw = 0.2  # rad
            # trans check
            self.assertAlmostEqual(d_trans[0], gt_d_trans[0], delta=delta_xy)
            self.assertAlmostEqual(d_trans[1], gt_d_trans[1], delta=delta_xy)
            self.assertAlmostEqual(d_trans[2], 0.0)

            # angle check
            self.assertAlmostEqual(d_euler[0], 0.0)
            self.assertAlmostEqual(d_euler[1], 0.0)
            self.assertAlmostEqual(d_euler[2], gt_d_euler[2], delta=delta_yaw)

        # Confirm to ensure that no topics were missed entirely
        self.assertNotEqual(self.first_gt, None)
        self.assertNotEqual(self.first_odom, None)

        if self.show_info:
            errs = np.abs(np.array([
                d_trans[0] - gt_d_trans[0],
                d_trans[1] - gt_d_trans[1],
                d_euler[2] - gt_d_euler[2]]))
            for i in range(len(max_errs)):
                if errs[i] > max_errs[i]:
                    max_errs[i] = errs[i]

            self._node.get_logger().info(
                'max_errs: x[m]:%.2f, y[m]:%.2f, theta[rad]:%.2f' %
                (max_errs[0], max_errs[1], max_errs[2]))
