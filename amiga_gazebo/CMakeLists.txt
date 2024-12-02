cmake_minimum_required(VERSION 3.5)
project(amiga_gazebo)


find_package(ament_cmake REQUIRED)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)


install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/worlds
)


ament_package()

