cmake_minimum_required(VERSION 3.8)
project(amiga_ros2_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(amiga_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(bt_runner src/bt.cpp)

add_executable(dummy_wp_server src/dummy_wp_server.cpp)

ament_target_dependencies(bt_runner
  rclcpp
  nav2_msgs
  behaviortree_ros2
  geometry_msgs
  geographic_msgs
  tf2
  tf2_geometry_msgs
  amiga_interfaces
)

ament_target_dependencies(dummy_wp_server
  rclcpp
  rclcpp_action
  amiga_interfaces
  nav2_msgs
  geometry_msgs
  geographic_msgs
)

install(TARGETS bt_runner
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS dummy_wp_server
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/)

# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/)

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
