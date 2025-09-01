cmake_minimum_required(VERSION 3.5)
project(mocap4r2_marker_viz)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(mocap4r2_marker_viz_srvs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(mocap4r2_marker_viz src/mocap4r2_marker_viz_main.cpp)

add_library(${PROJECT_NAME}_NODE src/mocap4r2_marker_viz_node.cpp)

target_include_directories(${PROJECT_NAME}_NODE
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_NODE rclcpp mocap4r2_msgs visualization_msgs geometry_msgs mocap4r2_marker_viz_srvs)

target_link_libraries(mocap4r2_marker_viz ${PROJECT_NAME}_NODE)

install(TARGETS
  ${PROJECT_NAME}_NODE
  mocap4r2_marker_viz
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
