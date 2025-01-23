from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle',
            executable='obstacle_detection_node',
            name='obstacle_detection_node',
            output='screen',
            parameters=[
                {'left_trigger_pin': 17},
                {'left_echo_pin': 27},
                {'right_trigger_pin': 22},
                {'right_echo_pin': 24},
                {'obstacle_threshold': 0.3},
            ]
        ),
        Node(
            package='obstacle',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen',
            parameters=[
                {'left_trigger_pin': 17},
                {'left_echo_pin': 27},
                {'right_trigger_pin': 22},
                {'right_echo_pin': 24},
            ]
        ),
    ])
