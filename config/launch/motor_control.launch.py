from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_motor_control',  # Replace with your ROS2 package name
            executable='motor_control_node',  # Node executable
            name='motor_control_node',
            parameters=[
                {'left_motor_pwm_pin': 13},
                {'left_motor_dir_pin': 16},
                {'right_motor_pwm_pin': 18},
                {'right_motor_dir_pin': 23}
            ]
        )
    ])
