cmake_minimum_required(VERSION 3.8)
project(diff_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/diff_bot
)

ament_package()
