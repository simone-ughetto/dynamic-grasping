# Copyright (c) 2023 José Miguel Guerrero Hernández
#
# Licensed under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) License;
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://creativecommons.org/licenses/by-sa/4.0/
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    mocap4r2_marker_viz_dir = get_package_share_directory('mocap4r2_marker_viz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_namespace = LaunchConfiguration('use_namespace')
    mocap4r2_system = LaunchConfiguration('mocap4r2_system')

    declare_mocap4r2_system = DeclareLaunchArgument(
        'mocap4r2_system',
        default_value='optitrack',
        description='')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            mocap4r2_marker_viz_dir, 'rviz', 'config.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    start_rviz_cmd = Node(
        condition=UnlessCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    start_mocap4r2_marker_rviz = Node(
        package='mocap4r2_marker_viz',
        executable='mocap4r2_marker_viz',
        output='both',
        emulate_tty=True,
        parameters=[{'mocap4r2_system': mocap4r2_system}],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_mocap4r2_system)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(start_rviz_cmd)
    ld.add_action(start_mocap4r2_marker_rviz)

    return ld
