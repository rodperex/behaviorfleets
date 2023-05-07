# Copyright 2023 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    sp_dir = get_package_share_directory('bt4_bumpgo')

    main_cmd = Node(
        package='bt4_bumpgo',
        executable='bt4_bumpgo',
        name='bt4_bumpgo',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        remappings=[
            ('input_scan', '/scan_raw'),
            ('output_vel', '/key_vel')
        ]
    )  

    ld = LaunchDescription()
    ld.add_action(main_cmd)

    return ld
