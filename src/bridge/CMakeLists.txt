cmake_minimum_required(VERSION 3.5)
project(bridge)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    src/websocket_client.py
    src/websocket_ros2_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install .env file
install(
  FILES src/.env
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
