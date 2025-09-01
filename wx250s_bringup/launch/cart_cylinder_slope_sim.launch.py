#!/usr/bin/env python3
"""
Custom launch file for WidowX 250s with a mobile cart and a separate 
graspable cylinder placed on top of the cart. There is also a slope that the cart can climb.

This file works identically to cart_cylinder_sim.launch.py but with the addition of a slope.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    PythonExpression,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # -------------------------------------------------------------------------
    # Retrieve Launch Configurations
    # -------------------------------------------------------------------------
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    hardware_type = LaunchConfiguration('hardware_type').perform(context)
    update_rate = LaunchConfiguration('update_rate')
    object_name = LaunchConfiguration('object_name')
    launch_rsp = LaunchConfiguration('launch_rsp')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # -------------------------------------------------------------------------
    # 1. Include MoveIt (for arm planning)
    # -------------------------------------------------------------------------
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('interbotix_xsarm_moveit'),
                         'launch', 'xsarm_moveit.launch.py')
        ]),
        launch_arguments={
            'robot_model': 'wx250s',
            'robot_name': 'wx250s',
            'use_moveit_rviz': use_rviz,
            'hardware_type': hardware_type,
            'launch_rsp': launch_rsp,
            'use_sim_time': use_sim_time,
            'transform_timeout': '2.0'
        }.items()
    )

    # -------------------------------------------------------------------------
    # 2. Include MoCap NatNet nodes
    # -------------------------------------------------------------------------
    mocap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('mocap4r2_optitrack_driver'),
                         'launch', 'optitrack2.launch.py')
        ]),
        launch_arguments={
            'robot_sub_topic': 'wx250s/pose',
            'robot_pub_topic': 'wx250s/twist',
            'object_sub_topic': 'red_cylinder/pose',
            'object_pub_topic': 'red_cylinder/twist',
        }.items(),
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'actual'"]))
    )

    # -------------------------------------------------------------------------
    # 3. WidowX Arm (to be launched later)
    # -------------------------------------------------------------------------
    robot_description_pkg = 'interbotix_xsarm_descriptions'
    standard_xacro_file = PathJoinSubstitution([
        get_package_share_directory(robot_description_pkg),
        'urdf', 'wx250s.urdf.xacro'
    ])
    arm_robot_description = {
        'robot_description': ParameterValue(Command([
            'xacro ', standard_xacro_file,
            ' robot_name:=wx250s',
            ' hardware_type:=', hardware_type,
            ' use_gripper:=true',
            ' show_gripper_bar:=true',
            ' show_gripper_fingers:=true',
            ' use_world_frame:=true'
        ]), value_type=str)
    }
    arm_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='wx250s',
        output='screen',
        parameters=[arm_robot_description]
    )

    # -------------------------------------------------------------------------
    # 4. Slope Description and Nodes
    # -------------------------------------------------------------------------
    slope_xacro = PathJoinSubstitution([
        get_package_share_directory('wx250s_bringup'),
        'urdf', 'slope.urdf.xacro'
    ])
    slope_name = 'slope'
    slope_description = {'robot_description': ParameterValue(Command(['xacro ', slope_xacro]), value_type=str)}
    slope_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='slope_state_publisher',
        namespace=slope_name,
        output='screen',
        parameters=[slope_description, {'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'gz_classic'"]))
    )
    spawn_slope = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_slope',
        namespace=slope_name,
        output='screen',
        arguments=[
            '-topic', '/' + slope_name + '/robot_description',
            '-entity', slope_name,
            '-x', '0.3',  # Move to correct X position
            '-y', '-0.7', # Move slope to y = -1.5
            '-z', '0.0',  # Base on the ground
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'   # No rotation - mesh Y-axis (length) aligns with world Y-axis
        ],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'gz_classic'"]))
    )

    # -------------------------------------------------------------------------
    # 5. Cart Description and Nodes (POSITIONED ON SLOPE)
    # -------------------------------------------------------------------------
    cart_xacro = PathJoinSubstitution([
        get_package_share_directory('wx250s_bringup'),
        'urdf', 'cart.urdf.xacro'
    ])
    cart_name = 'mobile_cart'
    cart_description = {'robot_description': ParameterValue(Command(['xacro ', cart_xacro]), value_type=str)}
    cart_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='cart_state_publisher',
        namespace=cart_name,
        output='screen',
        parameters=[cart_description, {'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'gz_classic'"]))
    )
    cart_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='cart_joint_state_publisher',
        namespace=cart_name,
        output='screen',
        parameters=[{
            'rate': 30,
            'publish_default_positions': True,
            'use_gui': False,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'gz_classic'"]))
    )
    # Cart positioned behind the slope at specified coordinates
    spawn_cart = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_cart',
        namespace=cart_name,
        output='screen',
        arguments=[
            '-topic', '/' + cart_name + '/robot_description',
            '-entity', cart_name,
            '-x', '0.3',            # Align with slope X position
            '-y', '-1.0',           # Behind the slope as requested
            '-z', '0.03',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '1.5708'          # 90 degrees rotation around Z-axis
        ],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'gz_classic'"]))
    )

    # -------------------------------------------------------------------------
    # 6. Cylinder Description and (Ephemeral) Spawn Node (ADJUSTED FOR SLOPE)
    # -------------------------------------------------------------------------
    cylinder_xacro = PathJoinSubstitution([
        get_package_share_directory('wx250s_bringup'),
        'urdf', 'cylinder.urdf.xacro'
    ])
    cylinder_name = 'red_cylinder'
    cylinder_description = {'robot_description': ParameterValue(Command(['xacro ', cylinder_xacro]), value_type=str)}
    # Launch the state publisher immediately (persistent node)
    cylinder_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='cylinder_state_publisher',
        namespace=cylinder_name,
        output='screen',
        parameters=[cylinder_description, {'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'gz_classic'"]))
    )
    # Cylinder positioned on cart
    spawn_cylinder = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_cylinder',
        namespace=cylinder_name,
        output='screen',
        arguments=[
            '-topic', '/' + cylinder_name + '/robot_description',
            '-entity', cylinder_name,
            '-x', '0.3',             # Align with cart
            '-y', '-1.0',            # Align with cart
            '-z', '0.085',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',             # No inclination
            '-reference_frame', 'world'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'gz_classic'"]))
    )

    # -------------------------------------------------------------------------
    # 7. Additional Nodes (Teleop, TF Transforms, Controllers, etc.)
    # -------------------------------------------------------------------------
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -title "Cart Teleop - CLICK HERE FIRST" -geometry 80x24+0+0 -bg black -fg green -e',
        remappings=[
            ('/cmd_vel', '/' + cart_name + '/cmd_vel'),
        ],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'gz_classic'"]))
    )
    # TF transforms adjusted for corrected positions
    cart_to_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cart_to_world_tf',
        output='screen',
        arguments=[
            '--frame-id', 'world',
            '--child-frame-id', 'odom',
            '--x', '0.4', '--y', '-2.0', '--z', '0.03',  # Updated cart position
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '1.5708'  # 90 degrees rotation
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'gz_classic'"]))
    )
    cylinder_to_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cylinder_to_world_tf',
        output='screen',
        arguments=[
            '--frame-id', 'world',
            '--child-frame-id', 'red_cylinder/world',
            '--x', '0.4', '--y', '-2.0', '--z', '0.085',  # Updated cylinder position
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'gz_classic'"]))
    )
    # Replace the existing relative_object_state_publisher_node with these two conditional nodes:
    fake_relative_object_state_publisher_node = Node(
        package='wx250s_bringup',
        executable='fake_relative_object_state_publisher',
        name='fake_relative_object_state_publisher_node',
        output='screen',
        parameters=[{
            'object_name': object_name,
            'robot_base_frame': "wx250s/base_link",
            'alpha': 0.5,
            'update_rate': update_rate
        }],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'gz_classic'"]))
    )

    relative_object_state_publisher_node = Node(
        package='wx250s_bringup',
        executable='relative_object_state_publisher',
        name='relative_object_state_publisher_node',
        output='screen',
        parameters=[{
            'object_name': object_name,
            'robot_base_frame': "wx250s/base_link",
        }],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'actual'"]))
    )

    # -------------------------------------------------------------------------
    # 8. Define Event Handlers for Sequential Execution
    # -------------------------------------------------------------------------
    # (a) When spawn_slope exits, trigger spawn_cart.
    spawn_slope_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_slope,
            on_exit=[spawn_cart]
        )
    )

    # (b) When spawn_cart exits, trigger spawn_cylinder.
    spawn_cart_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_cart,
            on_exit=[spawn_cylinder]
        )
    )

    # (c) When spawn_cylinder exits, trigger the remaining nodes:
    #     Teleop, TF transforms
    spawn_cylinder_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_cylinder,
            on_exit=[
                teleop_node,
                cart_to_world_tf,
                cylinder_to_world_tf,
                arm_state_publisher_node,
            ]
        )
    )

    # -------------------------------------------------------------------------
    # 9. Assemble the Launch Description
    # -------------------------------------------------------------------------
    # Replace individual nodes with the dynamic_grasping launch file
    dynamic_grasping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dynamic_grasping_controller'),
                       'launch', 'dynamic_grasping.launch.py')
        ]),
        launch_arguments={
            'hardware_type': hardware_type,
            'object_name': object_name,
            'update_rate': update_rate,
            'enable_trajectory_logging': 'true',
            'trajectory_log_path': '/home/sughetto/my_robot_ws/trajectory_data.csv'
        }.items()
    )

    # First, include all nodes that should run regardless of hardware_type
    launch_actions = [
        moveit_launch
    ]
    
    # Add hardware_type specific nodes
    if hardware_type == 'gz_classic':
        # Add simulation-specific nodes with sequential spawning
        launch_actions.extend([
            slope_state_publisher,
            spawn_slope,
            cart_state_publisher,
            cart_joint_state_publisher,
            cylinder_state_publisher,
            spawn_slope_event_handler,
            spawn_cart_event_handler,
            spawn_cylinder_event_handler,
            fake_relative_object_state_publisher_node
        ])
    else:
        # Add hardware-specific nodes
        launch_actions.extend([
            arm_state_publisher_node,
            mocap_launch,
            relative_object_state_publisher_node
        ])
    
    launch_actions.append(dynamic_grasping_launch)

    return launch_actions


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization if true'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'hardware_type',
            default_value='gz_classic',
            description='Launch hardware driver if "actual"; set false for simulation'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'update_rate',
            default_value='30.0',
            description='Rate at which to query Gazebo and publish object state (Hz)'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'object_name',
            default_value='red_cylinder',
            description='Name of the object to track in Gazebo'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rsp',
            default_value='false',
            description='Whether to launch robot_state_publisher in included launch files'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
