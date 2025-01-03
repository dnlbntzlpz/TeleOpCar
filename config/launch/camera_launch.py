from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_car',  # Replace with your package name
            executable='camera_node',  # Node executable name
            name='camera_node',  # Node name
            parameters=[
                {'front_left_camera': '/dev/video0'},
                {'front_right_camera': '/dev/video2'}
            ],
            output='screen'  # Log output to the terminal
        )
    ])
