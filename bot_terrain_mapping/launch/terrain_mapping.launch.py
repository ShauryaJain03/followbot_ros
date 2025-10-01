#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bot_terrain_pkg = get_package_share_directory('bot_terrain_mapping')
    
    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('lio_sam'), 
            '/launch/run.launch.py'
        ])
    )
    
    elevation_mapper_node = Node(
        package='bot_terrain_mapping',
        executable='elevation_mapper',
        name='elevation_mapper',
        parameters=[{
            'map_frame': 'odom',
            'robot_frame': 'base_footprint',
            'map_resolution': 0.1,
            'map_length': 30.0,
            'min_height': -1.0,
            'max_height': 3.0
        }],
        output='screen'
    )

    
    return LaunchDescription([
        lio_sam_launch,
        elevation_mapper_node,
    ])