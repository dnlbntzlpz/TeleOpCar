cmake_minimum_required(VERSION 3.5)
project(motors)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gpiozero REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    src/brake_manager_node.py
    src/motors_control_node.py
    src/servo_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
