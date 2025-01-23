cmake_minimum_required(VERSION 3.5)
project(obstacle)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gpiozero REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    src/obstacleDetection_node.py
    src/ultrasonic_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
