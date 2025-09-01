#!/usr/bin/env python3
"""
Hardware-only launch file for WidowX 250s to grasp moving objects.

This launch file is based on cart_cylinder_sim.launch.py but:
- Only works with hardware_type:=actual
- Excludes all Gazebo-related components (cart spawning, cylinder spawning, etc.)
- Excludes MoveIt components
- Includes only the essential hardware components

The file launches:
1. The robot hardware control (xsarm_ros_control)
2. MoCap tracking for object and robot
3. Relative object state publisher
4. Dynamic grasping controller

AFTER LAUNCHING, WAIT FOR THE ROBOT TO REACH THE REST POSE (ABOUT 5s) BEFORE SENDING ANY START COMMAND!
"""

import os

from ament_index_python.packages import get_package_share_directory
from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments,
    determine_use_sim_time_param,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # -------------------------------------------------------------------------
    # Retrieve Launch Configurations
    # -------------------------------------------------------------------------
    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    base_link_frame_launch_arg = LaunchConfiguration('base_link_frame')
    show_ar_tag_launch_arg = LaunchConfiguration('show_ar_tag')
    use_world_frame_launch_arg = LaunchConfiguration('use_world_frame')
    external_urdf_loc_launch_arg = LaunchConfiguration('external_urdf_loc')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')
    launch_rsp_launch_arg = LaunchConfiguration('launch_rsp')
    
    object_name = LaunchConfiguration('object_name')
    update_rate = LaunchConfiguration('update_rate')

    # sets use_sim_time parameter to 'false' for actual hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )

    # -------------------------------------------------------------------------
    # 1. Include robot hardware control (from xsarm_moveit.launch.py)
    # -------------------------------------------------------------------------
    xsarm_ros_control_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_ros_control'),
                'launch',
                'xsarm_ros_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'base_link_frame': base_link_frame_launch_arg,
            'show_ar_tag': show_ar_tag_launch_arg,
            'show_gripper_bar': 'true',
            'show_gripper_fingers': 'true',
            'use_world_frame': use_world_frame_launch_arg,
            'external_urdf_loc': external_urdf_loc_launch_arg,
            'use_rviz': 'false',
            'mode_configs': mode_configs_launch_arg,
            'hardware_type': hardware_type_launch_arg,
            'robot_description': robot_description_launch_arg,
            'use_sim_time': use_sim_time_param,
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
            'launch_rsp': launch_rsp_launch_arg,
        }.items(),
    )

    # -------------------------------------------------------------------------
    # 2. Include MoCap NatNet nodes (from cart_cylinder_sim.launch.py)
    # -------------------------------------------------------------------------
    mocap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('mocap4r2_optitrack_driver'),
                         'launch', 'optitrack2.launch.py')
        ]),
        launch_arguments={
            'robot_sub_topic': 'wx250s/pose',
            'robot_pub_topic': 'wx250s/twist',
            'object_sub_topic': f'{object_name.perform(context)}/pose',
            'object_pub_topic': f'{object_name.perform(context)}/twist',
        }.items(),
    )

    # -------------------------------------------------------------------------
    # 3. Relative object state publisher (from cart_cylinder_sim.launch.py)
    # -------------------------------------------------------------------------
    relative_object_state_publisher_node = Node(
        package='wx250s_bringup',
        executable='relative_object_state_publisher',
        name='relative_object_state_publisher_node',
        output='screen',
        parameters=[{
            'object_name': object_name,
            'robot_base_frame': "wx250s/base_link",
        }],
    )

    # -------------------------------------------------------------------------
    # 4. Dynamic grasping controller (from cart_cylinder_sim.launch.py)
    # -------------------------------------------------------------------------
    dynamic_grasping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dynamic_grasping_controller'),
                       'launch', 'dynamic_grasping.launch.py')
        ]),
        launch_arguments={
            'hardware_type': hardware_type_launch_arg,
            'object_name': object_name,
            'update_rate': update_rate,
            'enable_trajectory_logging': 'true',
            'trajectory_log_path': '/home/sughetto/my_robot_ws/trajectory_data.csv'
        }.items()
    )

    # -------------------------------------------------------------------------
    # 5. Delayed command to set robot to rest position
    # -------------------------------------------------------------------------
    delayed_rest_position_command = TimerAction(
        period=5.0,  # 10 seconds delay
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '-1', '/start_dynamic_grasping', 
                     'std_msgs/msg/Int8', 'data: 3'],
                name='rest_position_command',
                output='screen'
            )
        ]
    )

    # -------------------------------------------------------------------------
    # 6. Assemble the Launch Description
    # -------------------------------------------------------------------------
    return [
        xsarm_ros_control_launch_include,
        mocap_launch,
        relative_object_state_publisher_node,
        dynamic_grasping_launch,
        delayed_rest_position_command,
    ]


def generate_launch_description():
    declared_arguments = []
    
    # Arguments from xsarm_moveit.launch.py
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            default_value='wx250s',
            choices=get_interbotix_xsarm_models(),
            description='model type of the Interbotix Arm such as `wx200` or `rx150`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit'),
                'config',
                'modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'xs_driver_logging_level',
            default_value='INFO',
            choices=('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'),
            description='set the logging level of the X-Series Driver.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'hardware_type',
            default_value='actual',
            description='Launch hardware driver for actual robot'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rsp',
            default_value='true',
            choices=('true', 'false'),
            description='Launch robot_state_publisher if true.'
        )
    )
    
    # Arguments from cart_cylinder_sim.launch.py
    declared_arguments.append(
        DeclareLaunchArgument(
            'update_rate',
            default_value='30.0',
            description='Rate at which to query and publish object state (Hz)'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'object_name',
            default_value='purple_prism',
            description='Name of the object to track'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description='Use simulation time if true (should be false for actual hardware)'
        )
    )
    
    # Add the standard robot description arguments
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
            hardware_type='actual',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
