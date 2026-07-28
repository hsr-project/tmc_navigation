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

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
import launch_ros.actions


def generate_launch_description():
    base_angle_offset = DeclareLaunchArgument("base_angle_offset", default_value="0.0")
    wheel_radius_l = DeclareLaunchArgument("wheel_radius_l", default_value="0.1")
    wheel_radius_r = DeclareLaunchArgument("wheel_radius_r", default_value="0.1")
    caster_offset = DeclareLaunchArgument("caster_offset", default_value="0.0")
    vel_std = DeclareLaunchArgument("vel_std", default_value="[0.05, 0.05, 0.2]")
    valid_joint_vel_ths = DeclareLaunchArgument("valid_joint_vel_ths", default_value="[100.0, 100.0, 100.0]")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    imu_topic = DeclareLaunchArgument("imu_topic", default_value=TextSubstitution(text="data_raw"))
    joint_state_topic = DeclareLaunchArgument("joint_state_topic", default_value=TextSubstitution(text="joint_state"))
    imu_odom_topic = DeclareLaunchArgument("imu_odom_topic", default_value=TextSubstitution(text="odometry"))

    node = launch_ros.actions.Node(
        package='tmc_imu_wheel_odometry',
        executable='lower_imu_odom_node',
        name='lower_imu_odom', output='screen',
        parameters=[{'base_angle_offset': LaunchConfiguration('base_angle_offset')},
                    {'wheel_radius_l': LaunchConfiguration('wheel_radius_l')},
                    {'wheel_radius_r': LaunchConfiguration('wheel_radius_r')},
                    {'caster_offset': LaunchConfiguration('caster_offset')},
                    {'vel_std': LaunchConfiguration('vel_std')},
                    {'valid_joint_vel_ths': LaunchConfiguration('valid_joint_vel_ths')},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('imu', LaunchConfiguration('imu_topic')),
                    ('joint_state', LaunchConfiguration('joint_state_topic')),
                    ('imu_odom', LaunchConfiguration('imu_odom_topic'))])
    return LaunchDescription([
        base_angle_offset,
        wheel_radius_l,
        wheel_radius_r,
        caster_offset,
        vel_std,
        valid_joint_vel_ths,
        use_sim_time,
        imu_topic,
        joint_state_topic,
        imu_odom_topic,
        node])
