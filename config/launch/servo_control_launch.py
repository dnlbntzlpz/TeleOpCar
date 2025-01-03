from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_car',  # Replace with your package name
            executable='servo_control_node',  # Node executable name
            name='servo_control_node',  # Name for the node
            parameters=[
                {'servo_pwm_pin': 12}  # Pass the servo GPIO pin as a parameter
            ],
            output='screen'  # Output logs to the terminal
        )
    ])
