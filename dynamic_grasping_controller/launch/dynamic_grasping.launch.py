#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('dynamic_grasping_controller')
    
    # Path to the parameters YAML file
    params_file = os.path.join(package_dir, 'config', 'dynamic_planning_params.yaml')
    
    # Launch arguments
    hardware_type_arg = DeclareLaunchArgument(
        'hardware_type',
        default_value='gz_classic',
        description='Hardware type (gz_classic or actual)'
    )
    
    object_name_arg = DeclareLaunchArgument(
        'object_name',
        default_value='cylinder',
        description='Name of the object to track'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='30.0',
        description='Rate at which to query Gazebo and publish object state (Hz)'
    )
    
    enable_logging_arg = DeclareLaunchArgument(
        'enable_trajectory_logging',
        default_value='false',
        description='Whether to log trajectory data to file'
    )
    
    log_path_arg = DeclareLaunchArgument(
        'trajectory_log_path',
        default_value='~/trajectory_data.csv',
        description='Path to save trajectory log data'
    )
    
    enable_mocap_arg = DeclareLaunchArgument(
        'enable_mocap',
        default_value='false',
        description='Whether to launch the mocap system'
    )
    
    mocap_namespace_arg = DeclareLaunchArgument(
        'mocap_namespace',
        default_value='',
        description='Namespace for mocap nodes'
    )
    
    # Create nodes
    ee_pose_publisher_node = Node(
        package='dynamic_grasping_controller',
        executable='ee_pose_publisher_node',
        name='ee_pose_publisher_node',
        output='screen'
    )
    
    velocities_publisher_node = Node(
        package='dynamic_grasping_controller',
        executable='velocities_publisher_node',
        name='velocities_publisher_node',
        output='screen',
        parameters=[{
            'hardware_type': LaunchConfiguration('hardware_type')
        }]
    )
    
    trajectory_planner_node = Node(
        package='dynamic_grasping_controller',
        executable='trajectory_planner_node',
        name='trajectory_planner_node',
        output='screen',
        parameters=[
            params_file,
            {
                'hardware_type': LaunchConfiguration('hardware_type'),
                'object_name': LaunchConfiguration('object_name')
            }
        ]
    )
    
    gripper_planner_node = Node(
        package='dynamic_grasping_controller',
        executable='gripper_planner_node',
        name='gripper_planner_node',
        output='screen',
        parameters=[params_file]
    )
    
    dynamic_grasping_manager_node = Node(
        package='dynamic_grasping_controller',
        executable='dynamic_grasping_manager_node',
        name='dynamic_grasping_manager_node',
        output='screen',
        parameters=[
            params_file,
            {
                'hardware_type': LaunchConfiguration('hardware_type'),
                'object_name': LaunchConfiguration('object_name')
            }
        ]
    )
    
    
    # Mocap launch inclusion (conditional)
    mocap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('mocap4r2_optitrack_driver'),
                'launch',
                'optitrack2.launch.py'
            )
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('mocap_namespace'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_mocap'))
    )
    
    return LaunchDescription([
        hardware_type_arg,
        object_name_arg,
        update_rate_arg,
        enable_logging_arg,
        log_path_arg,
        enable_mocap_arg,
        mocap_namespace_arg,
        ee_pose_publisher_node,
        velocities_publisher_node,
        trajectory_planner_node,
        gripper_planner_node,
        dynamic_grasping_manager_node,
        mocap_launch
    ])