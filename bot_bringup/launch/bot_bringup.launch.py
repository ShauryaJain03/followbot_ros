import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bot_description = get_package_share_directory("bot_description")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(bot_description, "urdf", "bot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="empty",
        description="World file name (without .world extension)"
    )

    model = LaunchConfiguration("model")
    world_name = LaunchConfiguration("world_name")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bot_description,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'model': model,
            'world_name': world_name,
        }.items()
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


    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo,
        controller,

    ])
