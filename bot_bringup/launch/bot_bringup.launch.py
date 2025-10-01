from launch_ros.actions import Node
from launch import LaunchDescription
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="empty",
        description="World file name (without .world extension)"
    )

    world_name = LaunchConfiguration("world_name")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bot_description'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world_name': world_name}.items()
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bot_controller'),
                'launch',
                'controller.launch.py'
            )
        )
    )

    rule_engine = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bot_hri"),
                'launch',
                'rule_engine.launch.py'
            )
        )
    )

    battery_pub = Node(
        executable="mock_battery",
        package="bot_bringup",
        output="screen"
    )

    



    return LaunchDescription([
        world_name_arg,
        gazebo,
        controller,
        battery_pub,
    ])
