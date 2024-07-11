cmake_minimum_required(VERSION 3.8)
project(ackerman_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(cmd_vel2odom_node src/cmd_vel2odom_node.cpp)
add_executable(odom_to_tf_node src/odom_to_tf_node.cpp)

ament_target_dependencies(
  cmd_vel2odom_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(
  odom_to_tf_node
  rclcpp
  nav_msgs
  tf2
  tf2_ros
)

install(TARGETS 
  cmd_vel2odom_node
  odom_to_tf_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
