cmake_minimum_required(VERSION 3.5)

project(rqt_mocap4r2_control)

find_package(ament_cmake REQUIRED)

if(WIN32)
  message(STATUS "rqt_mocap4r2_control is not yet supported on Windows. Package will not be built.")
  ament_package()
  return()
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(mocap4r2_control REQUIRED)
find_package(mocap4r2_control_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  qt_gui_cpp
  rqt_gui_cpp
  mocap4r2_control
  mocap4r2_control_msgs
  sensor_msgs
)

set(rqt_mocap4r2_control_SRCS
  src/rqt_mocap4r2_control/mocap4r2_control.cpp
  src/rqt_mocap4r2_control/SystemController.cpp
)

set(rqt_mocap4r2_control_HDRS
  include/rqt_mocap4r2_control/mocap4r2_control.hpp
  include/rqt_mocap4r2_control/SystemController.hpp
)

set(rqt_mocap4r2_control_UIS
  src/rqt_mocap4r2_control/mocap4r2_control.ui
)

set(rqt_mocap4r2_control_INCLUDE_DIRECTORIES
  include
  ${Qt5Widgets_INCLUDE_DIRS}
)

qt5_wrap_cpp(rqt_mocap4r2_control_MOCS ${rqt_mocap4r2_control_HDRS})

qt5_wrap_ui(rqt_mocap4r2_control_UIS_H ${rqt_mocap4r2_control_UIS})

include_directories(
  ${rqt_mocap4r2_control_INCLUDE_DIRECTORIES}
)

add_library(${PROJECT_NAME} SHARED
  ${rqt_mocap4r2_control_SRCS}
  ${rqt_mocap4r2_control_MOCS}
  ${rqt_mocap4r2_control_UIS_H}
)

target_link_libraries(
  ${PROJECT_NAME}
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(PROGRAMS scripts/rqt_mocap4r2_control
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()