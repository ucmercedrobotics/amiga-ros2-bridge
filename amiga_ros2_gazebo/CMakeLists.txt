cmake_minimum_required(VERSION 3.8)
project(amiga_ros2_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  models
  meshes
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sim_canbus_twist.py
  scripts/sim_twist_control.py
  scripts/sim_gps_shim.py
  scripts/sim_imu_shim.py
  scripts/sim_camera_shim.py
  scripts/sim_joint_state_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
