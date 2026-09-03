cmake_minimum_required(VERSION 3.8)
project(odometry_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# Node 1: base_link_path_pub — publishes Path of base_link from TF
add_executable(base_link_path_pub src/base_link_path_pub.cpp)
ament_target_dependencies(base_link_path_pub
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Node 2: odom_to_baselink — transforms IMU odometries to base_link
add_executable(odom_to_baselink src/odom_to_baselink.cpp)
ament_target_dependencies(odom_to_baselink
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Node 3: simple_vio_bridge — bridges OpenVINS odometry to PX4
add_executable(simple_vio_bridge src/simple_vio_bridge.cpp)
ament_target_dependencies(simple_vio_bridge
  rclcpp nav_msgs px4_msgs)

install(TARGETS
  base_link_path_pub
  odom_to_baselink
  simple_vio_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
