from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="bot_hri",
            executable="rule_engine",
            output="screen"
        )
    ])