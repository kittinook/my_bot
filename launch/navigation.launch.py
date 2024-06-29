import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    package_name = "my_bot"
    config_dir = os.path.join(get_package_share_directory(package_name), 'config')
    rviz_config_dir = os.path.join(get_package_share_directory(package_name), 'rviz')
    map_dir = os.path.join(get_package_share_directory(package_name), 'maps')

    config_file = os.path.join(config_dir, 'mapper_params_online_sync.yaml')    
    rviz_config_file = os.path.join(rviz_config_dir, 'navigation.rviz')         # <<< edit
    map_file = LaunchConfiguration('map', default=os.path.join(map_dir, 'samplemap.yaml'))  # <<< edit
    param_file = LaunchConfiguration('params', default=os.path.join(config_dir, 'navigation_param.yaml'))

    world = os.path.join(get_package_share_directory(package_name), 'worlds', 'my_world.world') # <<< edit
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'yaml_filename': map_file}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to map file to load'
    )

    param_arg = DeclareLaunchArgument(
        'params',
        default_value=param_file,
        description='Full path to param file to load'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': param_file
        }.items(),
    )

    return LaunchDescription(
        [
            map_arg,
            param_arg,
            nav2,
            spawn_entity,
            gazebo,
            rsp,
            map_server,
            rviz
        ]
    )
