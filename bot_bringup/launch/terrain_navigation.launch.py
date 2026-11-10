from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('bot_description')
    nav2_share = get_package_share_directory('nav2_bringup')
    nav2_params = description_share + '/config/followbot_nav2.yaml'

    following_server = Node(
        package='opennav_following',
        executable='opennav_following',
        name='following_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}],
        remappings=[('cmd_vel', 'cmd_vel_smoothed')],
    )

    following_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_following',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['following_server'],
        }],
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_share + '/launch/bringup_launch.py'),
            launch_arguments={
                'use_sim_time': 'True',
                'slam': 'False',
                'use_localization': 'False',
                'autostart': 'True',
                'use_composition': 'False',
                'use_respawn': 'False',
                'params_file': nav2_params,
            }.items(),
        ),
        following_server,
        following_lifecycle_manager,
    ])
