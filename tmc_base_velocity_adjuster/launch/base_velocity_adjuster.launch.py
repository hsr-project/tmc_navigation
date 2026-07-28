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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
import launch_ros.actions


def generate_launch_description():
    base_frame = DeclareLaunchArgument(
        "base_frame", default_value=TextSubstitution(text="base_link"))
    enable_adjustment_default = DeclareLaunchArgument("enable_adjustment_default", default_value="true")
    obstacle = DeclareLaunchArgument(
        "obstacle", default_value=TextSubstitution(text="obstacle"))
    command_velocity = DeclareLaunchArgument(
        "command_velocity", default_value=TextSubstitution(text="command_velocity"))
    adjusted_velocity = DeclareLaunchArgument(
        "adjusted_velocity", default_value=TextSubstitution(text="adjusted_velocity"))

    params_file = os.path.join(get_package_share_directory('tmc_base_velocity_adjuster'),
                               'config', 'base_velocity_adjuster_config.yaml')

    node = launch_ros.actions.Node(
        package='tmc_base_velocity_adjuster',
        executable='base_velocity_adjuster',
        name='base_velocity_adjuster', output='screen',
        remappings=[('obstacle', LaunchConfiguration('obstacle')),
                    ('command_velocity', LaunchConfiguration('command_velocity')),
                    ('adjusted_velocity', LaunchConfiguration('adjusted_velocity'))],
        parameters=[params_file,
                    {'base_frame': LaunchConfiguration('base_frame')},
                    {'enable_adjustment_default': LaunchConfiguration('enable_adjustment_default')}]
    )
    return LaunchDescription([
        base_frame,
        enable_adjustment_default,
        obstacle,
        command_velocity,
        adjusted_velocity,
        node])
