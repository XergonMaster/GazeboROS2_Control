cmake_minimum_required(VERSION 3.8)
project(robots_description)


find_package(ament_cmake REQUIRED)

install(
  DIRECTORY common/urdf common/rviz
  DESTINATION share/${PROJECT_NAME}/common
)
install(
  DIRECTORY differential/urdf differential/rviz
  DESTINATION share/${PROJECT_NAME}/diff
)


ament_package()
