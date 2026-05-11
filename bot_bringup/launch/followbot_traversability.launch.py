import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bot_description = get_package_share_directory("bot_description")

    params_default = os.path.join(
        bot_description, "config", "traversability_baylands.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=params_default,
            description="Full path to the traversability parameter YAML file.",
        ),
        Node(
            package="traversability_gridmap",
            executable="traversability_node",
            name="traversability_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file"), {"use_sim_time": True}],
        ),
    ])
