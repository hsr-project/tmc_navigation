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
    reset_data_num = DeclareLaunchArgument("reset_data_num", default_value="3000")
    reset_bias_srv_timeout = DeclareLaunchArgument("reset_bias_srv_timeout", default_value="100")
    imu_angular_threshold = DeclareLaunchArgument("imu_angular_threshold", default_value="[0.02]")
    use_init_bias = DeclareLaunchArgument("use_init_bias", default_value="False")
    init_bias = DeclareLaunchArgument("init_bias", default_value="[0.0, 0.0, 0.0]")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    data_raw_topic = DeclareLaunchArgument("data_raw_topic", default_value=TextSubstitution(text="data_raw"))
    cmd_vel_topic = DeclareLaunchArgument("cmd_vel_topic", default_value=TextSubstitution(text="command_velocity"))
    data_raw_br_topic = DeclareLaunchArgument("data_raw_br_topic", default_value=TextSubstitution(text="data_br"))
    bias_topic = DeclareLaunchArgument("bias_topic", default_value=TextSubstitution(text="bias"))

    node = launch_ros.actions.Node(
        package='tmc_imu_wheel_odometry',
        executable='imu_reset_bias_node',
        name='bias_estimator', output='screen',
        parameters=[{'reset_data_num': LaunchConfiguration('reset_data_num')},
                    {'reset_srv_timeout': LaunchConfiguration('reset_bias_srv_timeout')},
                    {'imu_angular_threshold': LaunchConfiguration('imu_angular_threshold')},
                    {'use_init_bias': LaunchConfiguration('use_init_bias')},
                    {'init_bias': LaunchConfiguration('init_bias')},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('data_raw', LaunchConfiguration('data_raw_topic')),
                    ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
                    ('data_raw_br', LaunchConfiguration('data_raw_br_topic')),
                    ('bias', LaunchConfiguration('bias_topic'))])
    return LaunchDescription([
        reset_data_num,
        reset_bias_srv_timeout,
        imu_angular_threshold,
        use_init_bias,
        init_bias,
        use_sim_time,
        data_raw_topic,
        cmd_vel_topic,
        data_raw_br_topic,
        bias_topic,
        node])
