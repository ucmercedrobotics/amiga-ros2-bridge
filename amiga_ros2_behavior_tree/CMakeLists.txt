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
find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(amiga_interfaces REQUIRED)
find_package(LibXml2 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(bt_runner
  src/bt.cpp
  src/mission_tcp.cpp
  src/xml_validation.cpp
  src/actions/move_to_gps_location.cpp
  src/actions/move_to_relative_location.cpp
  src/actions/detect_object.cpp
  src/actions/sample_leaf.cpp
  src/actions/check_value.cpp
  src/actions/assert_true.cpp
)

add_executable(dummy_wp_server src/mocks/dummy_wp_server.cpp)

add_executable(dummy_ntp_server src/mocks/dummy_ntp_server.cpp)

add_executable(dummy_nav2_ntp_server src/mocks/dummy_nav2_ntp_server.cpp)

target_include_directories(bt_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bt_runner
  rclcpp
  behaviortree_ros2
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

ament_target_dependencies(dummy_ntp_server
  rclcpp
  rclcpp_action
  amiga_interfaces
)

ament_target_dependencies(dummy_nav2_ntp_server
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

target_link_libraries(bt_runner
  LibXml2::LibXml2
)

install(TARGETS bt_runner
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS dummy_wp_server
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS dummy_ntp_server
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS dummy_nav2_ntp_server
  DESTINATION lib/${PROJECT_NAME})

# Install public headers
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY schemas/
  DESTINATION share/${PROJECT_NAME}/schemas
)

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
