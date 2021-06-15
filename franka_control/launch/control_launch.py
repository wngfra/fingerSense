from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    commander_node = Node(
        package='finger_sense',
        executable='commander',
        parameters=[
            {'save_dir'  : './src/fingerSense/data'},
            {'mode'      : 'train'}
        ],
        output='screen',
        emulate_tty=True
    )

    signal_pub_node = Node(
        package='can_wifi',
        executable='tactile_signal_publisher'
    )

    franka_control_node = Node(
        package='franka_control',
        executable='node_launcher',
        arguments=['172.16.0.2']
    )

    ld.add_action(commander_node)
    ld.add_action(franka_control_node)
    ld.add_action(signal_pub_node)

    return ld
