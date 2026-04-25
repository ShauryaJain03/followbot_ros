import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    terrain_pkg = get_package_share_directory('bot_terrain_follower')


    follower_config = os.path.join(terrain_pkg, 'config', 'follower.yaml')
    traversability_config = os.path.join(
        terrain_pkg,
        'config',
        'traversability.yaml',
    )

    world_pose_topic = PythonExpression([
        "'/world/' + '", LaunchConfiguration('world_name'), "' + '/pose/info'"
    ])

    robot_ground_truth_publisher = Node(
        package='bot_terrain_follower',
        executable='robot_ground_truth_publisher',
        name='robot_ground_truth_publisher',
        parameters=[follower_config, {
            'use_sim_time': True,
            'world_pose_topic': world_pose_topic,
        }],
        output='screen',
    )


    traversability_analyzer = Node(
        package='bot_terrain_follower',
        executable='traversability_analyzer',
        name='traversability_analyzer',
        #parameters=[traversability_config, {'use_sim_time': True}],
        output='screen',
    )

    capability_follower = Node(
        package='bot_terrain_follower',
        executable='capability_aware_follower',
        name='capability_aware_follower',
        parameters=[follower_config, {'use_sim_time': True}],
        output='screen',
    )

    metrics_logger = Node(
        package='bot_terrain_follower',
        executable='demo_metrics_logger',
        name='demo_metrics_logger',
        parameters=[follower_config, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        robot_ground_truth_publisher,
        traversability_analyzer,
        #capability_follower,
        #metrics_logger,
    ])
