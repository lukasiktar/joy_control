cmake_minimum_required(VERSION 3.8)
project(joy_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#Find packages
find_package(ament_cmake REQUIRED)
#find_package(camera_calibration_parsers REQUIRED)
find_package(class_loader REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(tello_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  ${std_srvs_INCLUDE_DIRS}
  include
  h264decoder)



set(JOY_NODE_SOURCES
  src/joy_control_node.cpp)

set(JOY_NODE_DEPS
  class_loader
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs)

add_library(joy_control_node SHARED
  ${JOY_NODE_SOURCES})

target_compile_definitions(joy_control_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(joy_control_node
  ${JOY_NODE_DEPS})

rclcpp_components_register_nodes(joy_control_node "custom_joy::CustomJoyNode")
set(node_plugins "${node_plugins}custom_joy::CustomJoyNode;$<TARGET_FILE:joy_control_node>\n")

add_executable(joy_control_main
  src/joy_control_main.cpp
  ${JOY_NODE_SOURCES})

ament_target_dependencies(joy_control_main
  ${JOY_NODE_DEPS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Install nodes
install(
  TARGETS joy_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Install world and launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# Install executables
install(
  TARGETS joy_control_main 
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
