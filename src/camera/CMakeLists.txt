cmake_minimum_required(VERSION 3.5)
project(camera)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    src/camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
