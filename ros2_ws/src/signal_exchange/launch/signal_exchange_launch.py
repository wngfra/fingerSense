import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import ExecutableInPackage


def generate_launch_description():
    ld = LaunchDescription()

    # Prepare the Robot State Publisher node
    rsp_exe = ExecutableInPackage(package='signal_exchange', executable='robot_state_publisher')
    rsp_action = ExecuteProcess(
        cmd=[rsp_exe, '100.120.20.60'], output='screen'
    )

    # Prepare the Tactile Sensor Driver
    tsd_exe = ExecutableInPackage(package='signal_exchange', executable='tactile_sensor_driver')
    tsd_action = ExecuteProcess(
        cmd=[tsd_exe, 'Tactile Sensor Driver'], output='screen'
    )

    ld.add_action(rsp_action)
    ld.add_action(tsd_action)

    return ld
