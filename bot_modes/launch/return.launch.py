from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="bot_modes",
            executable="return_node",
            name="return_node",
            output="screen"
        )

    ])
