cmake_minimum_required(VERSION 3.8)
project(amiga_ros2_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
## example_interfaces no longer needed after custom srv
# find_package(example_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(amiga_interfaces REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(amiga_navigation_interfaces REQUIRED)
find_package(kortex_interfaces REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(bt_runner
  src/bt.cpp
  src/xml_validation.cpp
  src/actions/move_to_gps_location.cpp
  src/actions/move_to_tree_id.cpp
  src/actions/move_to_relative_location.cpp
  src/actions/detect_object.cpp
  src/actions/sample_leaf.cpp
  src/actions/check_value.cpp
  src/actions/assert_true.cpp
  src/actions/follow_person.cpp
)

add_executable(dummy_wp_server src/mocks/dummy_wp_server.cpp)

add_executable(dummy_ntp_server src/mocks/dummy_ntp_server.cpp)

add_executable(dummy_nav2_ntp_server src/mocks/dummy_nav2_ntp_server.cpp)

add_executable(dummy_segment_leaves_server src/mocks/dummy_segment_leaves_server.cpp)

add_executable(dummy_tree_id_server src/mocks/dummy_tree_id_server.cpp)

add_executable(orchard_management_node src/orchard_management.cpp)

add_executable(tcp_demux_node src/tcp_demux_node.cpp)

target_include_directories(bt_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bt_runner
  rclcpp
  behaviortree_ros2
  amiga_navigation_interfaces
  kortex_interfaces
  sensor_msgs
  std_msgs
)

ament_target_dependencies(dummy_wp_server
  rclcpp
  rclcpp_action
  amiga_navigation_interfaces
  nav2_msgs
  geometry_msgs
  geographic_msgs
)

ament_target_dependencies(dummy_ntp_server
  rclcpp
  rclcpp_action
  amiga_navigation_interfaces
)

ament_target_dependencies(dummy_nav2_ntp_server
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

ament_target_dependencies(dummy_segment_leaves_server
  rclcpp
  rclcpp_action
  kortex_interfaces
)

ament_target_dependencies(dummy_tree_id_server
  rclcpp
  rclcpp_action
  amiga_navigation_interfaces
)

ament_target_dependencies(orchard_management_node
  rclcpp
  amiga_interfaces
  std_msgs
)

ament_target_dependencies(tcp_demux_node
  rclcpp
  std_msgs
)

target_link_libraries(bt_runner
  LibXml2::LibXml2
)

target_link_libraries(orchard_management_node
  nlohmann_json::nlohmann_json
)

install(TARGETS bt_runner
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS dummy_wp_server
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS dummy_ntp_server
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS dummy_nav2_ntp_server
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS dummy_segment_leaves_server
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS dummy_tree_id_server
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS orchard_management_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS tcp_demux_node
  DESTINATION lib/${PROJECT_NAME})

# Install public headers
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY schemas/
  DESTINATION share/${PROJECT_NAME}/schemas
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

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
