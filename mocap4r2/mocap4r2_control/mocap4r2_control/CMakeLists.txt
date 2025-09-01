cmake_minimum_required(VERSION 3.5)
project(mocap4r2_control)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mocap4r2_control_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  mocap4r2_control_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/mocap4r2_control/ControlledLifecycleNode.cpp
  src/mocap4r2_control/ControllerNode.cpp
  src/mocap4r2_control/AuxiliarNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(auxiliar_main src/auxiliar_main.cpp)
ament_target_dependencies(auxiliar_main ${dependencies})
target_link_libraries(auxiliar_main ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  auxiliar_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(std_msgs REQUIRED)
  set(dependencies ${dependencies} std_msgs)

  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
