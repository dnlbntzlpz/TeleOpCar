cmake_minimum_required(VERSION 3.5)
project(bridge)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    src/websocket_ros2_bridge.py
    src/websocket_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install .env file
install(
  DIRECTORY launch/
  #  FILES src/.env
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
