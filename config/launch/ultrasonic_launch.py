from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_car',  # Replace with your package name
            executable='ultrasonic_node',  # Node executable name
            name='ultrasonic_node',  # Node name
            parameters=[
                {'left_trigger_pin': 17},
                {'left_echo_pin': 27},
                {'right_trigger_pin': 22},
                {'right_echo_pin': 24}
            ],
            output='screen'  # Log output to the terminal
        )
    ])
