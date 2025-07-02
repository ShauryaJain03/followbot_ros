from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        Node(
            package="bot_modes",
            executable="gps_waypoint_node",
            name="gps_waypoint_node",
            output="screen"
        )
    )