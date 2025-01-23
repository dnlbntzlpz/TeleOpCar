from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bridge',
            executable='websocket_ros2_bridge',
            name='websocket_ros2_bridge',
            output='screen',
            parameters=[{
                'host': os.getenv("HOST", "0.0.0.0"),
                'port': int(os.getenv("SOCKET_PORT", 8765))
            }]
        )
    ])
