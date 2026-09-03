cmake_minimum_required(VERSION 3.8)
project(amiga_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/LoRaFrame.msg
  srv/GetTreeInfo.srv
  srv/InterpretAnomaly.srv
  srv/InterpretNote.srv
  srv/VerifyReplan.srv
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
