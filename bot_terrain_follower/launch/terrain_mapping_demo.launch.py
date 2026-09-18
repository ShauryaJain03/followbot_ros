"""Launch the self-contained LiDAR terrain mapper in the Baylands simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    terrain_package = get_package_share_directory('bot_terrain_follower')
    bringup_package = get_package_share_directory('bot_bringup')
    rviz_config = os.path.join(terrain_package, 'rviz', 'terrain_mapping.rviz')

    world_name = LaunchConfiguration('world_name')
    use_rviz = LaunchConfiguration('use_rviz')
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_package, 'launch', 'bot_bringup.launch.py')),
        launch_arguments={'world_name': world_name, 'use_rviz': 'false'}.items(),
    )
    mapper = Node(
        package='bot_terrain_follower',
        executable='traversability_analyzer',
        name='traversability_analyzer',
        parameters=[os.path.join(terrain_package, 'config', 'traversability.yaml'),
                    {'use_sim_time': True}],
        output='screen',
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='terrain_rviz',
        arguments=['-d', rviz_config], parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz), output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='baylands'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        bringup,
        mapper,
        rviz,
    ])
