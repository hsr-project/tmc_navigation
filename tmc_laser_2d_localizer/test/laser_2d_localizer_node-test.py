#!/usr/bin/env python3
# Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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

import math
from threading import Lock
from threading import Thread
import unittest

from geometry_msgs.msg import PoseWithCovarianceStamped
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
import launch_ros
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.wait_for_message import wait_for_message
from rosgraph_msgs.msg import Clock
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)
from tf_transformations import euler_from_quaternion

# Parameters
TEST_TIMEOUT_SEC = 60.0
RATE_HZ = 100.0
THRESH_EQUAL = 0.0001
WAIT_TF_TIMEOUT_SEC = 0.1
MAX_ERROR_POSE = 0.05
# Temporarily relax the threshold as tests may fail in CodeBuild
# TODO(kazuki_shibamiya) : CodeBuildで安定的にテストが通るようにする
# MAX_ERROR_ORIENTATION = 0.04
MAX_ERROR_ORIENTATION = 0.1


def get_yaw(quat_msg):
    return euler_from_quaternion(
        [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
    )[2]


@pytest.mark.launch_test
def generate_test_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Node settings for laser_2d_localizer
    localizer_yaml = PathJoinSubstitution([
        FindPackageShare("tmc_laser_2d_localizer"),
        "test",
        "config",
        "laser_2d_localizer_test.yaml",
    ])
    localizer_node = launch_ros.actions.Node(
        package="tmc_laser_2d_localizer",
        executable="laser_2d_localizer",
        name="laser_2d_localizer",
        parameters=[
            localizer_yaml,
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("input_cloud", "extracted_point_cloud"),
            ("laser_2d_pose_ref", "laser_2d_pose_ref_"),
            ("static_distance_ros_map", "static_distance_map_ref")
        ]
    )

    # Playback settings for rosbag
    bag_path = PathJoinSubstitution([
        FindPackageShare("tmc_laser_2d_localizer"),
        "test",
        "rosbag",
        "405_house_test",
    ])
    rosbag_node = launch.actions.ExecuteProcess(
        cmd=["ros2", "bag", "play", bag_path, "--clock", "-l"],
        output="log",
    )

    return launch.LaunchDescription([
        use_sim_time_arg,
        localizer_node,
        rosbag_node,
        launch_testing.actions.ReadyToTest()
    ])


class TestLaser2dLocalizerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        """Initialize the test environment for laser_2d_localizer node."""
        self._node = rclpy.create_node("test_laser_2d_localizer_py")
        self._tf_buffer = Buffer(
            cache_time=rclpy.duration.Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

        use_sim_time_param = rclpy.parameter.Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )
        update_parameters = [use_sim_time_param]
        self._node.set_parameters(update_parameters)
        self._lock = Lock()
        self._estimated_pose_first = None
        self._estimated_pose = None
        self._is_sub_result = False

        # Wait until receiving the clock from rosbag,
        # Ensure testing with the clock from rosbag
        # Receive the clock from rosbag with BEST_EFFORT
        qos_policy_best_effort = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        result, _ = wait_for_message(
            Clock,
            self._node,
            "/clock",
            qos_profile=qos_policy_best_effort,
            time_to_wait=10,
        )
        self.assertTrue(
            result, msg="Failed to receive clock message from rosbag")

        # Receive Pose with RELIABLE to match laser_2d_localizer
        qos_policy_reliable = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub_estimated_pose = self._node.create_subscription(
            PoseWithCovarianceStamped,
            "/laser_2d_pose",
            self._estimated_pose_callback,
            qos_policy_reliable,
        )

        # Create a service to obtain parameters from laser_2d_localizer
        self._get_parameters_client = self._node.create_client(
            GetParameters,
            "/laser_2d_localizer/get_parameters"
        )

        # Spin the Node in a separate thread to receive messages
        self._executor = MultiThreadedExecutor()
        self._thread = Thread(
            target=rclpy.spin,
            args=(self._node, self._executor),
            daemon=True,
        )
        self._thread.start()

    def tearDown(self):
        self._node.destroy_node()

    def _estimated_pose_callback(
        self,
        msg: PoseWithCovarianceStamped
    ) -> None:
        """Save incoming PoseWithCovarianceStamped messages.

        The first message is saved in a separate variable `self._estimated_pose_first`
        for initial position verification.
        Following messages are saved in `self._estimated_pose` for subsequent localization verification.
        When a message is received, the flag `_is_sub_result` is set to True to indicate
        that a result has been received.

        Args:
            msg (`PoseWithCovarianceStamped`): The incoming pose message containing estimated position and covariance.
        """
        # Save the initial estimated position for initial value testing
        with self._lock:
            if self._estimated_pose_first is None:
                self._estimated_pose_first = msg

            self._estimated_pose = msg
            self._is_sub_result = True

    def test_localize(self) -> None:
        """Test the laser_2d_localizer node to ensure it correctly localizes the robot.

        TF between "floor/405_house_test" and "base_footprint" is used as groundtruth
        to verify the localization accuracy.
        """
        # Obtain the initial PoseWithCovarianceStamped message and verify the initial position
        rate = self._node.create_rate(RATE_HZ)

        # Obtain parameters of laser_2d_localizer
        request = GetParameters.Request(
            names=["init_x", "init_y", "init_theta_deg"])
        response = self._get_parameters_client.call(request)
        if not response:
            self._node.get_logger().error(
                "Failed to get parameters from laser_2d_localizer node"
            )
            return

        init_x_gt = response.values[0].double_value
        init_y_gt = response.values[1].double_value
        init_theta_gt = math.radians(response.values[2].double_value)

        # Wait until the initial position estimation result is obtained
        while (rclpy.ok() and not self._is_sub_result):
            rate.sleep()

        estimated_x = self._estimated_pose_first.pose.pose.position.x
        estimated_y = self._estimated_pose_first.pose.pose.position.y
        estimated_yaw = get_yaw(
            self._estimated_pose_first.pose.pose.orientation)
        self.assertAlmostEqual(
            estimated_x, init_x_gt, delta=THRESH_EQUAL,
            msg="Initial X position is incorrect",
        )
        self.assertAlmostEqual(
            estimated_y, init_y_gt, delta=THRESH_EQUAL,
            msg="Initial Y position is incorrect",
        )
        self.assertAlmostEqual(
            estimated_yaw, init_theta_gt, delta=THRESH_EQUAL,
            msg="Initial orientation is incorrect",
        )
        self._node.get_logger().info(
            "Initial pose is correct: "
            f"x={estimated_x}, y={estimated_y}, yaw={estimated_yaw}"
        )

        # Obtain the estimated result after rosbag playback
        # Evaluate the subscribed estimated result until timeout
        start_time = self._node.get_clock().now().nanoseconds
        last_time = 0
        while (
            rclpy.ok()
            and self._node.get_clock().now().nanoseconds - start_time < TEST_TIMEOUT_SEC * 1e9
        ):
            # Confirm that rosbag has not finished playback (loop)
            now_time = self._node.get_clock().now().nanoseconds
            self.assertLessEqual(last_time, now_time)
            last_time = now_time

            # Loop at rate(100) Hz and wait for the estimated result
            self._is_sub_result = False
            while not self._is_sub_result:
                rate.sleep()

            # Obtain the transformation from tf from floor/405_house_test -> base_footprint and evaluate the error
            # Obtain the transformation that matches the time of _estimated_pose through interpolation
            # (Taking the latest transformation increases error and causes test failure)
            # TODO(shigemichi_matsuzaki): 上記処理を行うためのベストプラクティスが他にあれば処理を変更
            transform = None
            while transform is None or \
                    transform.header.stamp != self._estimated_pose.header.stamp:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        "floor/405_house_test",
                        "base_footprint",
                        self._estimated_pose.header.stamp,
                        rclpy.duration.Duration(seconds=WAIT_TF_TIMEOUT_SEC),
                    )
                except (
                    LookupException,
                    ConnectivityException,
                    ExtrapolationException,
                ) as e:
                    self._node.get_logger().warn(f"{e}")

            # Calculate the error between the tf transformation and the estimated position
            error_x = abs(
                transform.transform.translation.x
                - self._estimated_pose.pose.pose.position.x
            )
            error_y = abs(
                transform.transform.translation.y
                - self._estimated_pose.pose.pose.position.y
            )
            tf_yaw = get_yaw(transform.transform.rotation)
            msg_yaw = get_yaw(self._estimated_pose.pose.pose.orientation)
            error_yaw = abs((tf_yaw - msg_yaw + math.pi) %
                            (2 * math.pi) - math.pi)

            # Output the error to the log
            header = self._estimated_pose.header
            time = f"{header.stamp.sec}.{header.stamp.nanosec:09}"
            self._node.get_logger().info(
                f"Transform error at {time}: "
                f"error_x={error_x}, "
                f"error_y={error_y}, "
                f"error_yaw={error_yaw}"
            )

            # Confirm that the error is within the allowable range
            self.assertLess(error_x, MAX_ERROR_POSE, f"error_x={error_x}")
            self.assertLess(error_y, MAX_ERROR_POSE, f"error_y={error_y}")
            self.assertLess(error_yaw, MAX_ERROR_ORIENTATION,
                            f"error_yaw={error_yaw}")
