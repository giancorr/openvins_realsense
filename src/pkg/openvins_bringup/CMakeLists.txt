cmake_minimum_required(VERSION 3.8)
project(openvins_bringup)

find_package(ament_cmake REQUIRED)

# Install config and launch directories
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
