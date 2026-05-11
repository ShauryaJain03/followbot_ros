import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bot_bringup = get_package_share_directory("bot_bringup")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world_name",
            default_value="baylands",
            description="World file name for the simulation launch.",
        ),
        DeclareLaunchArgument(
            "slam_start_delay",
            default_value="8.0",
            description="Delay before starting lidarslam so sim topics and TF are up.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz for lidarslam.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bot_bringup, "launch", "bot_bringup.launch.py")
            ),
            launch_arguments={
                "world_name": LaunchConfiguration("world_name"),
                "use_ground_truth_odom": "false",
                "use_rviz": "false",
            }.items(),
        ),
        TimerAction(
            period=LaunchConfiguration("slam_start_delay"),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bot_bringup, "launch", "followbot_lidarslam.launch.py")
                    ),
                    launch_arguments={
                        "use_sim_time": "true",
                        "use_rviz": LaunchConfiguration("use_rviz"),
                    }.items(),
                )
            ],
        ),
    ])
