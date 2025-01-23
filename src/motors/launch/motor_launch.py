from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motors',
            executable='brake_manager_node',
            name='brake_manager_node',
            output='screen',
        ),
        Node(
            package='motors',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen',
            parameters=[
                {'left_motor_pwm_pin': 13},
                {'left_motor_dir_pin': 16},
                {'right_motor_pwm_pin': 18},
                {'right_motor_dir_pin': 23},
            ]
        ),
        Node(
            package='motors',
            executable='servo_control_node',
            name='servo_control_node',
            output='screen',
            parameters=[
                {'servo_pwm_pin': 12},
            ]
        ),
    ])
