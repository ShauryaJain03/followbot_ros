from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_detector',
            executable='apriltag_detector_node',
            name='apriltag_detector',
            output='screen',
            parameters=[
                {'image_transport': 'raw'},
                {'tag_family': 'tf36h11'}  # Change to your tag family if needed
            ],
            remappings=[
                ('image', '/camera/image_raw'),           # Change if your camera topic is different
                ('camera_info', '/camera/camera_info'),   # Change if your camera info topic is different
                ('tags', '/apriltag_detections')
            ]
        )
    ])
