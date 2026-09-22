"""Run external traversability_mapping from LIO-SAM without duplicate TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    terrain_share = get_package_share_directory('bot_terrain_follower')
    mapping_share = get_package_share_directory('traversability_mapping_ros')
    ros_params = LaunchConfiguration('ros_params_file')
    core_params = LaunchConfiguration('core_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_params_file',
            default_value=os.path.join(terrain_share, 'config', 'lio_traversability_ros.yaml')),
        DeclareLaunchArgument(
            'core_params_file',
            default_value=os.path.join(terrain_share, 'config', 'lio_traversability_core.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='bot_terrain_follower',
            executable='lio_traversability_keyframe_bridge',
            name='lio_traversability_keyframe_bridge',
            parameters=[ros_params, {'use_sim_time': use_sim_time}],
            output='screen'),
        Node(
            package='traversability_mapping_ros',
            executable='global_traversability',
            name='global_traversability_node',
            parameters=[ros_params, {
                'parameter_file_path': core_params,
                'use_sim_time': use_sim_time,
            }],
            output='screen'),
        Node(
            package='traversability_mapping_ros',
            executable='local_traversability',
            name='local_traversability_node',
            parameters=[ros_params, {
                'parameter_file_path': core_params,
                'use_sim_time': use_sim_time,
            }],
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='traversability_rviz',
            arguments=['-d', os.path.join(mapping_share, 'rviz', 'traversability.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])
