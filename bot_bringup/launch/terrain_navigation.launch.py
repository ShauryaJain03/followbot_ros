from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    description_share = get_package_share_directory('bot_description')
    nav2_share = get_package_share_directory('nav2_bringup')

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
                'params_file': description_share + '/config/followbot_nav2.yaml',
            }.items(),
        ),
    ])
