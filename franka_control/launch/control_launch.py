from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    peception_agent_node = Node(
        package='finger_sense',
        executable='perception_agent',
        parameters=[
            {'core_dir'  : './src/fingerSense/finger_sense/finger_sense/core.csv'},
            {'factor_dir': './src/fingerSense/finger_sense/finger_sense/factors.npy'},
            {'save_dir'  : './src/fingerSense/preprocess/data/'},
            {'mode'      : 'train'}
        ],
        output='screen',
        emulate_tty=True
    )

    signal_pub_node = Node(
        package='can_wifi',
        executable='tactile_signal_publisher',
        parameters=[
            {'mode' : 'sensor'}
        ],
    )

    franka_control_node = Node(
        package='franka_control',
        executable='node_launcher',
        arguments=['172.16.0.2']
    )

    ld.add_action(peception_agent_node)
    ld.add_action(franka_control_node)
    ld.add_action(signal_pub_node)

    return ld
