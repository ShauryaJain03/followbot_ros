import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{'use_sim_time': True}]
    )


    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bot_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
        parameters=[{'use_sim_time': True}]
    )


    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
        ]
    )