from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bot_gps',
            executable='gps_waypoint_follower',
            name='gps_waypoint_follower'
        )
    ])
