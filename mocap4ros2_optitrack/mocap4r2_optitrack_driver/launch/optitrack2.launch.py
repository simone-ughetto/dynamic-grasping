# Copyright 2021 Institute for Robotics and Intelligent Machines,
#                Georgia Institute of Technology
# Copyright 2019 Intelligent Robotics Lab
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
#
# Author: Christian Llanes <christian.llanes@gatech.edu>
# Author: David Vargas Frutos <david.vargas@urjc.es>

import os

from ament_index_python.packages import get_package_share_directory
import launch

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import SetEnvironmentVariable,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare

import lifecycle_msgs.msg


def generate_launch_description():

    # Path to the driver config file
    driver_params_file_path = os.path.join(get_package_share_directory(
      'mocap4r2_optitrack_driver'), 'config', 'mocap4r2_optitrack_driver_params.yaml')

    # Path to the velocity estimator config file
    vel_estimator_params_file_path = PathJoinSubstitution([
        FindPackageShare('mocap4r2_optitrack_driver'),
        'config',
        'velocity_estimator_params.yaml'
    ])

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    driver_node = LifecycleNode(
        name='mocap4r2_optitrack_driver_node',
        namespace=LaunchConfiguration('namespace'),
        package='mocap4r2_optitrack_driver',
        executable='mocap4r2_optitrack_driver_main',
        output='screen',
        parameters=[LaunchConfiguration('driver_config_file')], # Renamed for clarity
    )

    # Make the driver node take the 'configure' transition
    driver_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the driver node take the 'activate' transition
    driver_activate_trans_event = EmitEvent(
       event = ChangeState(
           lifecycle_node_matcher = launch.events.matchers.matches_action(driver_node),
           transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )


    # Add rigid body publisher node
    rigid_body_publisher_node = Node(
        package='mocap4r2_optitrack_driver',
        executable='rigid_body_publisher_node',
        name='rigid_body_publisher_node',
        output='screen',
        namespace=LaunchConfiguration('namespace')
    )

    # Object 1 velocity estimator node (e.g., for the red_cylinder)
    object_vel_estimator_node = Node(
        package='mocap4r2_optitrack_driver',
        executable='velocity_estimator_node',
        name='object_vel_estimator_node', # Name must match the YAML file
        output='screen',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            # Load parameters from the YAML file
            LaunchConfiguration('vel_estimator_config_file'),
            # Override/set specific parameters not in the file (like topics)
            {
                "subscription_topic": LaunchConfiguration('object_sub_topic'),
                "publication_topic": LaunchConfiguration('object_pub_topic'),
            }
        ]
    )

    # Object 2 velocity estimator node (e.g., for the wx250s)
    robot_vel_estimator_node = Node(
        package='mocap4r2_optitrack_driver',
        executable='velocity_estimator_node',
        name='robot_vel_estimator_node', # Name must match the YAML file
        output='screen',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            # Load parameters from the YAML file
            LaunchConfiguration('vel_estimator_config_file'),
            # Override/set specific parameters not in the file (like topics)
            {
                "subscription_topic": LaunchConfiguration('robot_sub_topic'),
                "publication_topic": LaunchConfiguration('robot_pub_topic'),
            }
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    # --- Declare Launch Arguments ---
    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(DeclareLaunchArgument('driver_config_file', default_value=driver_params_file_path))
    # Declare argument for the new velocity estimator config file
    ld.add_action(DeclareLaunchArgument('vel_estimator_config_file', default_value=vel_estimator_params_file_path))

    # Arguments for topic names (still useful)
    ld.add_action(DeclareLaunchArgument('object_sub_topic', default_value='/red_cylinder/pose',
                                       description='Subscription topic for the first object velocity estimator'))
    ld.add_action(DeclareLaunchArgument('object_pub_topic', default_value='/red_cylinder/twist',
                                       description='Publication topic for the first object velocity estimator'))
    ld.add_action(DeclareLaunchArgument('robot_sub_topic', default_value='/wx250s/pose',
                                       description='Subscription topic for the second object velocity estimator'))
    ld.add_action(DeclareLaunchArgument('robot_pub_topic', default_value='/wx250s/twist',
                                       description='Publication topic for the second object velocity estimator'))

    # REMOVE old sigma arguments - they are now in the YAML file
    # ld.add_action(DeclareLaunchArgument('sigma_pos', default_value='8.0e-4', ...))
    # ld.add_action(DeclareLaunchArgument('sigma_accel', default_value='3.0e-2', ...))
    # ld.add_action(DeclareLaunchArgument('sigma_ang_accel', default_value='2.0e-2', ...))

    # --- Add Actions ---
    ld.add_action(driver_node)
    ld.add_action(driver_configure_trans_event)
    ld.add_action(driver_activate_trans_event)
    ld.add_action(rigid_body_publisher_node)
    ld.add_action(object_vel_estimator_node)
    ld.add_action(robot_vel_estimator_node)

    return ld
