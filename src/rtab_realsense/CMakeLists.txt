cmake_minimum_required(VERSION 3.5)
project(my_realsense_package)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()