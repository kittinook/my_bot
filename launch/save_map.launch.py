from pathlib import Path

from launch_ros.actions.node import ExecuteProcess, Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable


def generate_launch_description():
    mkdir_maps = ExecuteProcess(
        cmd=[
            FindExecutable(name='mkdir'),
            ' -p ',
            str(Path.home())+'/maps'
        ],
        shell=True
    )
    map_saver_cli = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver_cli',
        output='screen',
        arguments=['-f', str(Path.home())+'/maps/samplemap'],
        parameters=[{'save_map_timeout': 10000.0}])

    delay_map_saver_cli_after_mkdir_maps = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mkdir_maps,
            on_exit=[map_saver_cli]
        )
    )
    return LaunchDescription([
        mkdir_maps,
        delay_map_saver_cli_after_mkdir_maps
    ])