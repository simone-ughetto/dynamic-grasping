# Dynamic grasping for WidowX-250s

## Overview

This repository contains code and launch/configuration used to enable the robot arm WidowX 250 S to grasp objects that move with a constant linear velocity relative to the robot base. The system supports:
- Grasping stationary objects as well as objects that move with a constant linear velocity (including cases where the arm moves and the object is stationary relative to the base).
- Online adaptation and replanning after a change in the object motion, provided the motion remains linear and constant and the replanning criteria are satisfied.

## Packages included

- `dynamic_grasping_controller` — Implementation of the dynamic grasping control system, with custom trajectory planning, dynamic grasping manager (finite-state machine), custom kinematic functions library and other tools.
- `wx250s_bringup` — Custom bringup package with various launch files, both for simulations in Gazebo and hardware tests.
- Several utility/driver packages from Interbotix: `interbotix_ros_core`, `interbotix_ros_manipulators`, `interbotix_ros_toolboxes`, `moveit_visual_tools` (four packages from the [Interbotix GitHub](https://github.com/Interbotix)). Minor changes are present to  better adapt them to the dynamic grasping setup.
- `IFRA_LinkAttacher` — package for attaching/detaching objects in Gazebo, useful to fake grasps in simulations ([IFRA_LinkAttacher GitHub](https://github.com/IFRA-Cranfield/IFRA_LinkAttacher)).
- `mocap4r2`, `mocap_msgs`, `mocap4ros2_optitrack` are packages from [Optitrack MoCap4ROS2 Setup GitHub](https://github.com/OptiTrack/mocap4ros2_optitrack) and are needed to stream pose information from the computer that processes MoCap data.

## Important configuration notes

The main configuration file for planning and runtime parameters is `config/dynamic_planning_params.yaml` in the `dynamic_grasping_controller` package:
- In that YAML file some parameters are specially marked:
  - Parameters marked with a red circle (🔴) must be set carefully every time after changing the object to grasp or switching between simulations and hardware tests, in order to guarantee a correct functioning of the control system.
  - Parameters marked with a yellow circle (🟡) should be tuned as needed for either simulation or hardware tests to improve the overall functionality (they affect thresholds, offsets and durations). Just pay attention to the offsets when switching objects to grasp, especially for those with elongated shapes.
- Always ensure the `object_name` in the YAML and any object/topic names you provide to the system match the object published by your simulation or mocap system.

## Simulation

Simulations use Gazebo Classic to run and test the system. Before running, verify that `object_name` and the object pose publisher used by the simulated world publish the same frame/namespace.
- For a simple simulation with a cart and a red cylinder on top of it (stationary arm), launch the file: `ros2 launch dynamic_grasping_controller cart_cylinder_sim.launch.py`.
- For a simulation with an additional slope (fixed), launch: `ros2 launch dynamic_grasping_controller cart_cylinder_slope_sim.launch.py`.
- When in simulation, remember to set the parameter `predict_ahead` to `false`!

## Hardware tests

For hardware tests you must provide a source of real-time object and robot arm pose information. This setup uses the packages subscribes to topic into which pose data from a Motion Capture system are streamed, processed by other ROS 2 packages (like those from [Optitrack MoCap4ROS2 Setup GitHub](https://github.com/OptiTrack/mocap4ros2_optitrack)). The dynamic grasping system reads these poses to predict motion and plan the intercept.
- Make sure the mocap topic/frame and `object_name` in `config/dynamic_planning_params.yaml` are consistent.
- Make sure to connect the device that is running these packages to the same network of the computer processing MoCap data and save the device's IP address in the field `local_address` of the file `/mocap4ros2_optitrack/mocap4r2_optitrack_driver/config/mocap4r2_optitrack_driver_params.yaml`.
- Configure the OptiTrack system with the correct rigid body names matching your `object_name` parameter.
- To run hardware tests, connect the robot arm WidowX 250 S and launch the file: `ros2 launch dynamic_grasping_controller hardware_grasping.launch.py`.
- Make sure `predict_ahead` is set to `true` if the grasp fails because the robot is laggy.

## Dependencies and Installation

These packages work with ROS 2 Humble and Gazebo Classic (Gazebo 11). These underlying platforms are assumed to be already installed.

Install the additional required packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-diff-drive-controller \
  ros-humble-forward-command-controller \
  ros-humble-velocity-controllers \
  ros-humble-position-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-plugins \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-urdf \
  ros-humble-urdfdom
```

Source your workspace after building:

```bash
colcon build --symlink-install
source install/setup.bash
```
