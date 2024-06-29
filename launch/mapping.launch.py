import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    config_dir = os.path.join(get_package_share_directory('my_bot'), 'config')
    config_file = os.path.join(config_dir, 'mapper_params_online_sync.yaml')

    rviz_config_dir = os.path.join(get_package_share_directory('my_bot'), 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'mapping.rviz')

    map = os.path.join(
        get_package_share_directory('my_bot'),
        'worlds',
        'my_world.world')
    
    package_name = "my_bot"
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time":"true"}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py"
                )
            ]
        ),
        launch_arguments={
            'world': map
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "my_bot"
        ],
        output = "screen"
    )

    slam = Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='sync_slam_toolbox_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, config_file])

    rviz =    Node(
            package='rviz2',
            executable='rviz2',
            name='sync_slam_toolbox_node',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}])
    
    # Launch!
    return LaunchDescription(
        [   
            spawn_entity,
            gazebo,
            rsp,
            slam,
            rviz
        ]
    )
