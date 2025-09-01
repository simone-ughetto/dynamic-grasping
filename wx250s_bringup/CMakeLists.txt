##############################################################################
# C++ Settings
##############################################################################
cmake_minimum_required(VERSION 3.22.1)  
project(wx250s_bringup)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_compile_options(
  -O3 -march=native -ffast-math     
  -fno-semantic-interposition       
)

##############################################################################
# Find Dependencies
##############################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate message interfaces locally
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Get typesupport target for use in this package
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

##############################################################################
# Add Executable for the Gazebo Object State Publisher
##############################################################################
# Add executables for both publishers
add_executable(fake_relative_object_state_publisher src/fake_relative_object_state_publisher.cpp)
add_executable(relative_object_state_publisher src/relative_object_state_publisher.cpp)

# Include directories for both
target_include_directories(fake_relative_object_state_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(relative_object_state_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies for fake_relative_object_state_publisher
ament_target_dependencies(fake_relative_object_state_publisher
  rclcpp
  gazebo_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  std_msgs
  rosidl_runtime_cpp
)
target_link_libraries(fake_relative_object_state_publisher
  "${cpp_typesupport_target}"
)

# Link dependencies for relative_object_state_publisher
ament_target_dependencies(relative_object_state_publisher
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  rosidl_runtime_cpp
  tf2             # Add tf2 dependency
  tf2_ros         # Add tf2_ros dependency
  tf2_geometry_msgs # Add tf2_geometry_msgs dependency
)
target_link_libraries(relative_object_state_publisher
  "${cpp_typesupport_target}"
  Eigen3::Eigen # Link Eigen3 library
)

##############################################################################
# Install Targets
##############################################################################
# Install both executables
install(TARGETS
  fake_relative_object_state_publisher
  relative_object_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration, launch, and URDF files
install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(
  rosidl_default_runtime
  geometry_msgs
  std_msgs
)

ament_package()
