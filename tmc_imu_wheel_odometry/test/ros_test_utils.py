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

from copy import deepcopy
from threading import Event, Lock, Thread
import time

from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from sensor_msgs.msg import Imu
import tf_transformations


def time_to_sec(time):
    return time.nanoseconds / 1e+9


def get_mat_pose_diff(source, target):
    u"""Calculate the difference between two msgs (Odometry or PoseStamped)

    Args:
        source: Reference msg (Odometry or PoseStamped)
        target: Target msg

    Returns:
        Difference in translation vector [x, y, z], difference in Euler angles [roll, pitch, yaw]
    """
    def get_mat_from_msg(msg):
        u"""Calculate the transformation matrix from msg (Odometry or PoseStamped)

        Args:
            msg: The msg to be transformed

        Raises:
            NotImplementedError: The msg type is neither Odometry nor PoseStamped

        Returns:
            Transformation matrix (np.array 4x4)
        """
        if type(msg) is PoseStamped:
            pose = msg.pose
        elif type(msg) is Odometry:
            pose = msg.pose.pose
        else:
            raise NotImplementedError

        q = pose.orientation
        rot_mat = tf_transformations.quaternion_matrix(
            [q.x, q.y, q.z, q.w])
        trans = pose.position
        trans_mat = tf_transformations.translation_matrix(
            [trans.x, trans.y, trans.z])

        return np.dot(trans_mat, rot_mat)

    source_mat = get_mat_from_msg(source)
    target_mat = get_mat_from_msg(target)
    diff_mat = np.dot(np.linalg.inv(source_mat), target_mat)

    return tf_transformations.translation_from_matrix(diff_mat), \
        tf_transformations.euler_from_matrix(diff_mat, 'rxyz')


# Connection check for subscription
def check_subscription_connection(node, subscription, timeout=10.0):
    return wait_until(node, lambda: True if subscription.get_publisher_count() > 0 else False, timeout)


# Connection check for publisher
def check_publisher_connection(node, publisher, timeout=10.0):
    return wait_until(node, lambda: True if publisher.get_subscription_count() > 0 else False, timeout)


class ThreadHelper(object):
    def __init__(self, node, user_func, rate_hz):
        self._node = node
        if not callable(user_func):
            raise TypeError('Input for ThreadHelper should be callable.')
        if type(rate_hz) is not int and type(rate_hz) is not float:
            raise TypeError('Rate of thread should be value.')
        if rate_hz <= 0.0:
            raise ValueError('Rate of thread should be larger than 0.0')

        self._user_func = user_func
        self._sleeper = self._node.create_rate(rate_hz)

        self._thread = None
        self._event = Event()

    def __del__(self):
        self.stop()

    def start(self):
        if self._thread is None:
            self._event.clear()
            self._thread = Thread(target=self._loop)
            self._thread.daemon = True
            self._thread.start()
        else:
            self._node.get_logger().warn('ThreadHelper is already running.')

    def stop(self):
        if self._thread is not None:
            self._event.set()
            self._thread.join()
            self._thread = None

    def _loop(self):
        while True:
            if self._event.is_set():
                self._node.get_logger().info('ThreadHelper stop request')
                break
            self._user_func()
            self._sleeper.sleep()


class VariableImu(object):
    def __init__(self, node, topic='imu', frame_id='imu', rate_hz=100.0):
        self._node = node
        self._msg = Imu()
        self._msg.header.frame_id = frame_id
        self._last_publish_time = None
        self._yaw = 0.0

        self._pub = self._node.create_publisher(Imu, topic, 10)

        self._locker = Lock()

        self._thread_helper = ThreadHelper(self._node, self._publish, rate_hz)

    def publish_once(self):
        self._publish()

    def start(self):
        self._thread_helper.start()

    def stop(self):
        self._thread_helper.stop()

    def __del__(self):
        self.stop()

    def set_angular_velocities(self, x=0.0, y=0.0, z=0.0):
        with self._locker:
            self._msg.angular_velocity.x = x
            self._msg.angular_velocity.y = y
            self._msg.angular_velocity.z = z

    def set_linear_accelerations(self, x=0.0, y=0.0, z=0.0):
        with self._locker:
            self._msg.linear_acceleration.x = x
            self._msg.linear_acceleration.y = y
            self._msg.linear_acceleration.z = z

    def check_connection(self):
        return check_publisher_connection(self._node, self._pub)

    def _publish(self):
        now = self._node.get_clock().now()
        if self._last_publish_time is None:
            self._last_publish_time = now

        with self._locker:
            dt = time_to_sec(now) - time_to_sec(self._last_publish_time)
            self._last_publish_time = now

            self._yaw += self._msg.angular_velocity.z * dt

            quat = tf_transformations.quaternion_from_euler(0, 0, self._yaw)
            self._msg.orientation.x = quat[0]
            self._msg.orientation.y = quat[1]
            self._msg.orientation.z = quat[2]
            self._msg.orientation.w = quat[3]

            self._msg.header.stamp = now.to_msg()

            self._pub.publish(self._msg)


class VariableTwist(object):
    def __init__(self, node, topic='twist', rate_hz=100.0):
        self._node = node
        self._msg = Twist()
        self._pub = self._node.create_publisher(Twist, topic, 1)
        self._locker = Lock()
        self._thread_helper = ThreadHelper(self._node, self._publish, rate_hz)

    def __del__(self):
        self.stop()

    def publish_once(self):
        self._publish()

    def start(self):
        self._thread_helper.start()

    def stop(self):
        self._thread_helper.stop()

    def set_linear(self, x=0., y=0., z=0.):
        with self._locker:
            self._msg.linear.x = x
            self._msg.linear.y = y
            self._msg.linear.z = z

    def set_angular(self, x=0., y=0., z=0.):
        with self._locker:
            self._msg.angular.x = x
            self._msg.angular.y = y
            self._msg.angular.z = z

    def check_connection(self):
        return check_publisher_connection(self._node, self._pub)

    def _publish(self):
        with self._locker:
            self._pub.publish(self._msg)


class VariableJoint(object):
    def __init__(self, node, topic='joint', rate_hz=100.0):
        self._node = node
        self._msg = JointTrajectoryControllerState()
        self._msg.feedback.velocities = [0.0, 0.0, 0.0]
        self._msg.feedback.positions = [0.0, 0.0, 0.0]
        self._last_publish_time = None

        self._pub = self._node.create_publisher(
            JointTrajectoryControllerState, topic, 1)

        self._locker = Lock()

        self._thread_helper = ThreadHelper(self._node, self._publish, rate_hz)

    def __del__(self):
        self.stop()

    def publish_once(self):
        self._publish()

    def start(self):
        self._thread_helper.start()

    def stop(self):
        self._thread_helper.stop()
        self._last_publish_time = None

    def set_joint_velocities(self, v_l=0.0, v_r=0.0, v_roll=0.0):
        with self._locker:
            self._msg.feedback.velocities[0] = v_l
            self._msg.feedback.velocities[1] = v_r
            self._msg.feedback.velocities[2] = v_roll

    def set_joint_positions(self, p_l=0.0, p_r=0.0, p_roll=0.0):
        with self._locker:
            self._msg.feedback.positions[0] = p_l
            self._msg.feedback.positions[1] = p_r
            self._msg.feedback.positions[2] = p_roll

    def check_connection(self):
        return check_publisher_connection(self._node, self._pub)

    def _publish(self):
        now = self._node.get_clock().now()
        if self._last_publish_time is None:
            self._last_publish_time = now

        with self._locker:
            dt = time_to_sec(now) - time_to_sec(self._last_publish_time)
            self._last_publish_time = now

            # Integration
            self._msg.feedback.positions = \
                [pos + vel * dt for pos, vel in zip(
                    self._msg.feedback.positions, self._msg.feedback.velocities)]

            self._msg.header.stamp = now.to_msg()
            self._pub.publish(self._msg)


class TestIO(object):
    def __init__(self, node):
        self._node = node
        self._imu = None
        self._bias = None
        self._lock = Lock()

        # publisher
        self._pub_imu = VariableImu(
            self._node, topic='data_raw', rate_hz=100.0)
        self._pub_cmd_vel = VariableTwist(
            self._node, topic='command_velocity', rate_hz=100.0)

        # Subscribers
        self._sub_imu = self._node.create_subscription(
            Imu, 'data_br', self._update_imu, 10)
        self._sub_bias = self._node.create_subscription(
            Vector3, 'bias', self._update_bias, 10)

    def _update_imu(self, msg):
        with self._lock:
            self._imu = msg

    def _update_bias(self, msg):
        with self._lock:
            self._bias = msg

    def connection_check(self):
        # Sometimes the topic published immediately after creating a publisher/subscriber cannot be received
        # Add a wait at the start of the test to stabilize the test
        # TODO(kazuki_shibamiya) : トピックが漏れなく受信できるようになったらwaitを削除する
        time.sleep(1.0)
        if not self._pub_imu.check_connection():
            raise Exception('Failed to connect imu subscriber')
        if not self._pub_cmd_vel.check_connection():
            raise Exception('Failed to connect command vel subscriber')
        if not check_subscription_connection(self._node, self._sub_imu):
            raise Exception('Failed to connect imu publisher')
        if not check_subscription_connection(self._node, self._sub_bias):
            raise Exception('Failed to connect bias publisher')

    def imu(self):
        with self._lock:
            imu = deepcopy(self._imu)
        return imu

    def bias(self):
        with self._lock:
            bias = deepcopy(self._bias)
        return bias

    def publish_once(self):
        self._pub_imu.publish_once()
        self._pub_cmd_vel.publish_once()

    def start_publish(self):
        self._pub_imu.start()
        self._pub_cmd_vel.start()

    def stop_publish(self):
        self._pub_imu.stop()
        self._pub_cmd_vel.stop()

    def set_test_data(
            self,
            imu_angular=[0., 0., 0.],
            imu_linear=[0., 0., 0.],
            command_vel_angular=[0., 0., 0.],
            command_vel_linear=[0., 0., 0.]):
        self._pub_imu.set_angular_velocities(*imu_angular)
        self._pub_imu.set_linear_accelerations(*imu_linear)
        self._pub_cmd_vel.set_angular(*command_vel_angular)
        self._pub_cmd_vel.set_linear(*command_vel_linear)


def wait_until(node, func, timeout_sec=5.0):
    start = node.get_clock().now()
    timeout = rclpy.duration.Duration(seconds=timeout_sec)
    sleeper = node.create_rate(10.0)
    while node.get_clock().now() - start < timeout:
        if func():
            return True
        sleeper.sleep()

    return False
