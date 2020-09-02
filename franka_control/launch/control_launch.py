from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_wifi',
            namespace='finger_sense',
            executable='tactile_signal_publisher'
        ),
        Node(
            package='franka_control',
            namespace='finger_sense',
            executable='mafia'
        )   
    ])
