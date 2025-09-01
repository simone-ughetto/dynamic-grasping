cmake_minimum_required(VERSION 3.8)
project(mocap4r2_robot_gt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(mocap4r2_robot_gt_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_components
    mocap4r2_msgs
    mocap4r2_robot_gt_msgs
    tf2_ros
    geometry_msgs
    tf2_geometry_msgs
)

include_directories(
  include
)

add_library(gt_component SHARED src/mocap4r2_robot_gt/gt_component.cpp)
ament_target_dependencies(gt_component ${dependencies})
rclcpp_components_register_nodes(gt_component "mocap4r2_robot_gt::GTNode")

add_executable(gt_program src/gt_program.cpp)
ament_target_dependencies(gt_component ${dependencies})
target_link_libraries(gt_program gt_component)

add_library(set_gt_component SHARED src/mocap4r2_robot_gt/set_gt_component.cpp)
ament_target_dependencies(set_gt_component ${dependencies})
rclcpp_components_register_nodes(set_gt_component "mocap4r2_robot_gt::SetGTNode")

add_executable(set_gt_cli src/set_gt_cli.cpp)
ament_target_dependencies(set_gt_cli ${dependencies})
target_link_libraries(set_gt_cli set_gt_component)

install(TARGETS
  gt_component
  gt_program
  set_gt_component
  set_gt_cli
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
