from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    signal_pub_node = Node(
        package='can_wifi',
        executable='tactile_signal_publisher',
    )

    franka_control_node = Node(
        package='franka_control',
        executable='mafia'
    )

    ld.add_action(signal_pub_node)
    ld.add_action(franka_control_node)

    return ld