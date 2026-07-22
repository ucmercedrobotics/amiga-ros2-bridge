cmake_minimum_required(VERSION 3.8)
project(amiga_navigation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(amiga_navigation_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
)

add_executable(lidar_object_navigator
  src/lidar_object_navigator.cpp
)

ament_target_dependencies(lidar_object_navigator
  rclcpp
  rclcpp_action
  amiga_navigation_interfaces
  nav2_msgs
  sensor_msgs
  geometry_msgs
  Eigen3
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(navigate_to_pose_in_frame
  src/navigate_to_pose_in_frame.cpp
)

ament_target_dependencies(navigate_to_pose_in_frame
  rclcpp
  rclcpp_action
  amiga_navigation_interfaces
  nav2_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(linear_velo
  src/linear_velo.cpp
)

ament_target_dependencies(linear_velo
  rclcpp
  rclcpp_action
  amiga_navigation_interfaces
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  lidar_object_navigator
  navigate_to_pose_in_frame
  linear_velo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

FILE(GLOB SCRIPTS scripts/*.py)
install(PROGRAMS
  ${SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

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

ament_package()
