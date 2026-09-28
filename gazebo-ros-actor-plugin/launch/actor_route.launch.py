"""Manually publish a bounded, constant-speed actor route in Baylands."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('cmd_path_delay', default_value='2.0'),
        DeclareLaunchArgument('num_waypoints', default_value='10'),
        DeclareLaunchArgument('start_x', default_value='-73.090393'),
        DeclareLaunchArgument('start_y', default_value='-108.385361'),
        DeclareLaunchArgument('start_z', default_value='0.0'),
        DeclareLaunchArgument('waypoint_spacing', default_value='1.0'),
        DeclareLaunchArgument('heading', default_value='1.57079632679'),
        DeclareLaunchArgument('publish_repetitions', default_value='5'),
        DeclareLaunchArgument('republish_period', default_value='0.2'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    parameter_names = (
        'cmd_path_delay', 'num_waypoints', 'start_x', 'start_y', 'start_z',
        'waypoint_spacing', 'heading', 'publish_repetitions',
        'republish_period', 'use_sim_time',
    )
    parameters = {name: LaunchConfiguration(name) for name in parameter_names}

    return LaunchDescription(arguments + [
        Node(
            package='gazebo_ros_actor_plugin',
            executable='path_publisher.py',
            name='actor_path_publisher',
            parameters=[parameters],
            output='screen',
        ),
    ])
