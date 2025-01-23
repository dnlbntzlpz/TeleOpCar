from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {'front_left_camera': '/dev/video0'},
                {'front_right_camera': '/dev/video2'}
            ]
        )
    ])
