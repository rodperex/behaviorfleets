# Copyright 2023 Rodrigo Pérez-Rodríguez
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
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    sp_dir = get_package_share_directory('behaviorfleets')

    params = os.path.join(
        sp_dir,
        'params',
        'remote_config.yaml'
    )

    remote_cmd = Node(
        package='behaviorfleets',
        executable='remoteconfig',
        name='remote',
        output='screen',
        # parameters=[params],
        arguments=['test_1.yaml']
    )

    # Create the launch description and populate
    ld = LaunchDescriptbion()

    ld.add_action(remote_cmd)

    return ld
