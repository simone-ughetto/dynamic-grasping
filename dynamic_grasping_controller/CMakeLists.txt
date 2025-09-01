##############################################################################
# C++ Settings
##############################################################################
cmake_minimum_required(VERSION 3.22.1)  
project(dynamic_grasping_controller)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_compile_options(
  -O3 -march=native -ffast-math     
  -fno-semantic-interposition       
)

##############################################################################
# Dependencies
##############################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(wx250s_bringup REQUIRED)
find_package(linkattacher_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Threads REQUIRED)

# Ruckig library for trajectory generation (check if it's available)
find_package(ruckig QUIET)

##############################################################################
# widowx_kinematics_library (shared library)
##############################################################################
add_library(${PROJECT_NAME}_kinematics SHARED
  src/widowx_kinematics_library.cpp
  src/trajectory_utils.cpp
)

target_include_directories(${PROJECT_NAME}_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_kinematics
  rclcpp
  Eigen3
  trajectory_msgs
  geometry_msgs
  gazebo_msgs         # Add this dependency
  ament_index_cpp     # Add this dependency
  gazebo_ros          # Add this dependency
)

##############################################################################
# Node executables
##############################################################################
# 1) ee_pose_publisher_node
add_executable(ee_pose_publisher_node
  src/ee_pose_publisher_node.cpp
)

target_include_directories(ee_pose_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ee_pose_publisher_node
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  trajectory_msgs
  control_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  interbotix_xs_msgs
  Eigen3
)

target_link_libraries(ee_pose_publisher_node
  ${PROJECT_NAME}_kinematics
  "${cpp_typesupport_target}"
)

# 2) velocities_publisher_node
add_executable(velocities_publisher_node
  src/velocities_publisher_node.cpp
)

target_include_directories(velocities_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(velocities_publisher_node
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  trajectory_msgs
  control_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  interbotix_xs_msgs
  Eigen3
)

target_link_libraries(velocities_publisher_node
  ${PROJECT_NAME}_kinematics
  "${cpp_typesupport_target}"
)

# 3) trajectory_planner_node
add_executable(trajectory_planner_node
  src/trajectory_planner_node.cpp
)

target_include_directories(trajectory_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(trajectory_planner_node
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  trajectory_msgs
  control_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  interbotix_xs_msgs
  Eigen3
  moveit_ros_planning_interface
  wx250s_bringup # Add missing dependency
  ament_index_cpp
  gazebo_msgs
  gazebo_ros
)

target_link_libraries(trajectory_planner_node
  ${PROJECT_NAME}_kinematics
  "${cpp_typesupport_target}"
  Threads::Threads  # Add this line to link against pthread
  # Removed ${wx250s_bringup_TARGETS} as it's handled by ament_target_dependencies
)

# Link ruckig if available
if(ruckig_FOUND)
  target_link_libraries(trajectory_planner_node ruckig::ruckig)
endif()

# 5) dynamic_grasping_manager_node (Renamed from dynamic_grasping_manager)
add_executable(dynamic_grasping_manager_node
  src/dynamic_grasping_manager_node.cpp
)

target_include_directories(dynamic_grasping_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS} # Ensure Eigen includes are available
)

ament_target_dependencies(dynamic_grasping_manager_node
  rclcpp
  geometry_msgs
  std_msgs
  trajectory_msgs # Add missing dependency
  wx250s_bringup # Dependency for ObjectState msg
  linkattacher_msgs # Dependency for AttachLink srv
  Eigen3
)

target_link_libraries(dynamic_grasping_manager_node
  ${PROJECT_NAME}_kinematics # Add this line
  Eigen3::Eigen # Link against Eigen
  "${cpp_typesupport_target}"
)

# 6) gripper_planner_node (C++ implementation of gripper planner)
add_executable(gripper_planner_node
  src/gripper_planner_node.cpp
)

target_include_directories(gripper_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gripper_planner_node
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  std_msgs
)

target_link_libraries(gripper_planner_node
  "${cpp_typesupport_target}"
)



##############################################################################
# Install Targets
##############################################################################
install(
  TARGETS
    ${PROJECT_NAME}_kinematics
    ee_pose_publisher_node
    velocities_publisher_node
    trajectory_planner_node
    dynamic_grasping_manager_node # Use corrected name
    gripper_planner_node  # Add the new node here
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS
    scripts/visualize_trajectory.py
    scripts/fake_object_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

##############################################################################
# Export
##############################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_kinematics)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  trajectory_msgs
  control_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  interbotix_xs_msgs
  Eigen3
  linkattacher_msgs
  wx250s_bringup # Add wx250s_bringup to export dependencies
  ament_index_cpp
  gazebo_msgs
  gazebo_ros
)

# Must be after all the other ament_* calls
ament_package()
